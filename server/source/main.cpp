#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <cerrno>
#include <signal.h>

#include <jsoncpp/json/json.h>
#include <sstream>
#include <httplib.h>

#include <mqueue.h>
#include <fcntl.h>
#include <unistd.h>

#include <common.h>

#include <fstream>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <exception>

std::atomic<bool> running{true};
httplib::Server http_server;

mqd_t mq_mission_req;
mqd_t mq_mission_res;
mqd_t mq_signal_control;

Json::Value amr_seer_state;
std::mutex mutex_seer_state;

Mission_Response_Structure mission_response_save;
std::mutex lock_mission_response_save;

AMR_Hardware_Info_Structure amr_hardware;
std::mutex mutex_amr_hardware;

Mission_Active_Info_Structure mission_active = {0};
std::mutex mutex_mission_active;

SharedBlockAMRState* shm_amr_state = nullptr;
int shm_amr_state_fd = -1;

ROBOT_STATE robot_state = ROBOT_STATE_NONE;
WORK_MODE amr_work_mode = MANUAL;

void ensure_log_dir(const std::string& path) {
    if (mkdir(path.c_str(), 0755) != 0 && errno != EEXIST) {
        throw std::runtime_error("Cannot create log directory");
    }
}

void log_json(const std::string& json_str) {
    try {
        const std::string log_dir = "/home/esatech/log";
        ensure_log_dir(log_dir);

        std::time_t now = std::time(nullptr);
        std::tm tm_now{};
        localtime_r(&now, &tm_now);

        std::ostringstream file_name;
        file_name << log_dir << "/"
                  << std::put_time(&tm_now, "%Y-%m-%d")
                  << ".log";

        std::ostringstream timestamp;
        timestamp << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");

        std::ofstream log_file(file_name.str(), std::ios::app);
        if (!log_file.is_open()) {
            throw std::runtime_error("Cannot open log file");
        }

        log_file << "[" << timestamp.str() << "] : "
                 << json_str << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "[LOG ERROR] " << e.what() << std::endl;
    }
}

void terminal_sig_callback(int) 
{ 
    running = false; 

}

void cleanup() 
{
    if (shm_amr_state && shm_amr_state != MAP_FAILED) 
        munmap(shm_amr_state, sizeof(SharedBlockAMRState));
    if (shm_amr_state_fd >= 0) 
        close(shm_amr_state_fd);
    std::cout << "cleanup done (shm kept)\n";
}

bool wait_and_attach_amr_state() 
{
    while (running) 
    {
        shm_amr_state_fd = shm_open(SHM_AMR_STATE_NAME, O_RDWR, 0666);
        if (shm_amr_state_fd < 0) 
        {
            std::cerr << "waiting for SHM (writer)...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        // check size
        struct stat st;
        if (fstat(shm_amr_state_fd, &st) != 0) 
        { 
            perror("fstat"); 
            close(shm_amr_state_fd); 
            shm_amr_state_fd = -1;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        if (st.st_size < sizeof(SharedBlockAMRState)) 
        {
            std::cerr << "SHM exists but not sized yet, wait...\n";
            close(shm_amr_state_fd);
            shm_amr_state_fd = -1;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        shm_amr_state = (SharedBlockAMRState*)mmap(nullptr, sizeof(SharedBlockAMRState),
                                                        PROT_READ | PROT_WRITE, MAP_SHARED, shm_amr_state_fd, 0);
        if (shm_amr_state == MAP_FAILED) 
        { 
            perror("mmap"); 
            close(shm_amr_state_fd); 
            shm_amr_state_fd = -1;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        // wait until writer sets magic
        while (running && shm_amr_state->magic != SHM_AMR_STATE_MAGIC) 
        {
            std::cerr << "SHM present but not initialized. waiting...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        std::cout << "attached shm amr state(version=" << shm_amr_state->version << ")\n";
        return true;
    }
    return false;
}

void thread_read_state_amr_func()
{
    while (true)
    {
        /* code */
        // acquire rc_mutex
        int r;
        r = pthread_mutex_lock(&shm_amr_state->rw_mutex);
        if (r == EOWNERDEAD) 
        {
            std::cerr << "shm amr state EOWNERDEAD on rw_mutex, recovering...\n";
            pthread_mutex_consistent(&shm_amr_state->rw_mutex); 
        }
        AMR_State_Structure p;
        memcpy(&p,shm_amr_state->data,sizeof(AMR_State_Structure));
        pthread_mutex_unlock(&shm_amr_state->rw_mutex);
        //
        {
            std::lock_guard<std::mutex> lock(mutex_seer_state);
            Json::Value root;
            Json::CharReaderBuilder reader;
            std::string errs;
            std::string seer_json(p.seer_state,strnlen(p.seer_state, SEER_STATE_SIZE));
            std::istringstream ss(seer_json);
            bool ok = Json::parseFromStream(reader, ss, &root, &errs);
            if (!ok || !root.isObject()) 
            {
                std::cerr << "Parse error: " << errs << std::endl;
            }
            else
            {
                amr_seer_state["valid"] = true;
                amr_seer_state["state"] = root;
            }
        }

        robot_state = p.robot_state;

        {
            std::lock_guard<std::mutex> lock(mutex_mission_active);
            robot_state = p.robot_state;
            mission_active.mission_code = p.mission_active_info.mission_code;
            mission_active.action_index = p.mission_active_info.action_index;
            mission_active.mission_state = p.mission_active_info.mission_state;
            mission_active.mission_error_code = p.mission_active_info.mission_error_code;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_amr_hardware);
            amr_hardware.lift_state = p.lift_state;
            amr_hardware.comidity_state = p.commodity_state;
        }
        // std::cout << "1\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void http_server_init()
{
    /* init http server */
    http_server.Post("/work_mode", [](const httplib::Request& req, httplib::Response& res) {
        std::cout << "raw data : " << req.body << "\n";
        std::istringstream ss(req.body);
        Json::CharReaderBuilder rbuilder;
        Json::Value root;
        std::string errs;
        std::cout << "send work mode\n";
        if (!Json::parseFromStream(rbuilder, ss, &root, &errs)) {
            res.status = 400;
            res.set_content("{\"error\":\"Invalid JSON\"}", "application/json");
            return;
        }
        
        if (!root["work_mode"].isBool() || robot_state != ROBOT_STATE_IDLE) 
        {
            res.status = 400;
            res.set_content("{\"error\":\"missing or invalid key: work_mode\"}", "application/json");
            return;
        }

        amr_work_mode = (WORK_MODE)root["work_mode"].asBool();

        Json::Value reply;
        reply["response"] = 0;
        Json::StreamWriterBuilder writer;
        res.status = 200;
        res.set_content(Json::writeString(writer, reply), "application/json");
    });

    // POST /cancel  --> receive JSON, return JSON
    http_server.Post("/cancel", [](const httplib::Request& req, httplib::Response& res) {
        std::cout << "send cancel\n";

        if (robot_state == ROBOT_STATE_IDLE) 
        {
            res.status = 400;
            res.set_content("{\"error\":\"robot idle\"}", "application/json");
            return;
        }
        
        AMR_Signal_Control_Structure amr_signal_control = {0};
        amr_signal_control.signal_mission.signal_cancel = true;
        Json::Value reply;
        Json::StreamWriterBuilder writer;
        if(mq_send(mq_signal_control,reinterpret_cast<char*>(&amr_signal_control),sizeof(AMR_Signal_Control_Structure),0) == -1)
        {
            if (errno == EAGAIN) 
            {
                // queue full → drop hoặc overwrite
            }
            reply["response"] = 1;
        }
        else
        {
            reply["response"] = 0;
        }
        res.status = 200;
        res.set_content(Json::writeString(writer, reply), "application/json");
    });

    // POST /resume  --> receive JSON, return JSON
    http_server.Post("/resume", [](const httplib::Request& req, httplib::Response& res) {
        std::cout << "send resume\n";

        if (robot_state == ROBOT_STATE_IDLE) 
        {
            res.status = 400;
            res.set_content("{\"error\":\"robot idle\"}", "application/json");
            return;
        }
        
        AMR_Signal_Control_Structure amr_signal_control = {0};
        amr_signal_control.signal_mission.signal_resume_manual = true;
        Json::Value reply;
        Json::StreamWriterBuilder writer;
        if(mq_send(mq_signal_control,reinterpret_cast<char*>(&amr_signal_control),sizeof(AMR_Signal_Control_Structure),0) == -1)
        {
            if (errno == EAGAIN) 
            {
                // queue full → drop hoặc overwrite
            }
            reply["response"] = 1;
        }
        else
        {
            reply["response"] = 0;
        }
        res.status = 200;
        res.set_content(Json::writeString(writer, reply), "application/json");
    });

    // POST /pause  --> receive JSON, return JSON
    http_server.Post("/pause", [](const httplib::Request& req, httplib::Response& res) {
        std::cout << "send pause\n";

        if (robot_state == ROBOT_STATE_IDLE) 
        {
            res.status = 400;
            res.set_content("{\"error\":\"robot idle\"}", "application/json");
            return;
        }
        
        AMR_Signal_Control_Structure amr_signal_control = {0};
        amr_signal_control.signal_mission.signal_pause_manual = true;
        Json::Value reply;
        Json::StreamWriterBuilder writer;
        if(mq_send(mq_signal_control,reinterpret_cast<char*>(&amr_signal_control),sizeof(AMR_Signal_Control_Structure),0) == -1)
        {
            if (errno == EAGAIN) 
            {
                // queue full → drop hoặc overwrite
            }
            reply["response"] = 1;
        }
        else
        {
            reply["response"] = 0;
        }
        res.status = 200;
        res.set_content(Json::writeString(writer, reply), "application/json");
    });

    // POST /lift  --> receive JSON, return JSON
    http_server.Post("/lift", [](const httplib::Request& req, httplib::Response& res) {
        std::cout << "raw data : " << req.body << "\n";
        std::istringstream ss(req.body);
        Json::CharReaderBuilder rbuilder;
        Json::Value root_recv;
        std::string errs;
        bool check_json = Json::parseFromStream(rbuilder, ss, &root_recv, &errs);
        if (!check_json || !root_recv.isObject()) 
        {
            res.status = 400;
            res.set_content("{\"error\":\"Invalid JSON\"}", "application/json");
            return;
        }
        
        if(!root_recv["level"].isBool() || robot_state != ROBOT_STATE_IDLE || amr_work_mode != MANUAL)
        {
            res.status = 400;
            res.set_content("{\"error\":\"missing or invalid key: lift or robot is not idle or robot is auto\"}", "application/json");
            return;
        }

        AMR_Signal_Control_Structure amr_signal_control = {0};
        if(root_recv["level"].asBool())
        {
            amr_signal_control.signal_lift = LIFT_LEVEL_1;
        }
        else
        {
            amr_signal_control.signal_lift = LIFT_LEVEL_0;
        }
        Json::Value reply;
        Json::StreamWriterBuilder writer;
        if(mq_send(mq_signal_control,reinterpret_cast<char*>(&amr_signal_control),sizeof(AMR_Signal_Control_Structure),0) == -1)
        {
            if (errno == EAGAIN) 
            {
                // queue full → drop hoặc overwrite
            }
            reply["response"] = 1;
        }
        else
        {
            reply["response"] = 0;
        }
        res.status = 200;
        res.set_content(Json::writeString(writer, reply), "application/json");
    });

    http_server.Post("/relocation", [](const httplib::Request& req, httplib::Response& res) {
        std::cout << "raw data : " << req.body << "\n";
        std::istringstream ss(req.body);
        Json::CharReaderBuilder rbuilder;
        Json::Value root_recv;
        std::string errs;
        std::cout << "send relocation\n";
        bool check_json = Json::parseFromStream(rbuilder, ss, &root_recv, &errs);
        if (!check_json || !root_recv.isObject()) 
        {
            res.status = 400;
            res.set_content("{\"error\":\"Invalid JSON\"}", "application/json");
            return;
        }
        
        if(!root_recv["x"].isDouble() || !root_recv["y"].isDouble() || !root_recv["angle"].isDouble() || !root_recv["length"].isDouble()
            || robot_state != ROBOT_STATE_IDLE || amr_work_mode != MANUAL)
        {
            res.status = 400;
            res.set_content("{\"error\":\"missing or invalid key or robot is not idle or robot is auto\"}", "application/json");
            return;
        }

        AMR_Signal_Control_Structure amr_signal_control = {0};
        amr_signal_control.signal_relocation.valid = true;
        amr_signal_control.signal_relocation.x = root_recv["x"].asDouble();
        amr_signal_control.signal_relocation.y = root_recv["y"].asDouble();
        amr_signal_control.signal_relocation.angle = root_recv["angle"].asDouble();
        amr_signal_control.signal_relocation.length = root_recv["length"].asDouble();
        Json::Value reply;
        Json::StreamWriterBuilder writer;
        if(mq_send(mq_signal_control,reinterpret_cast<char*>(&amr_signal_control),sizeof(AMR_Signal_Control_Structure),0) == -1)
        {
            if (errno == EAGAIN) 
            {
                // queue full → drop hoặc overwrite
            }
            reply["response"] = 1;
        }
        else
        {
            reply["response"] = 0;
        }
        res.status = 200;
        res.set_content(Json::writeString(writer, reply), "application/json");
    });

    // POST /dispatch_mission  --> receive JSON, return JSON
    http_server.Post("/dispatch", [](const httplib::Request& req, httplib::Response& res) {
        std::istringstream ss(req.body);
        Json::CharReaderBuilder rbuilder;
        Json::Value root_recv;
        std::string errs;

        bool check = Json::parseFromStream(rbuilder, ss, &root_recv, &errs);
        if (!check || !root_recv.isObject()) 
        {
            res.status = 400;
            res.set_content("{\"error\":\"Invalid JSON\"}", "application/json");
            return;
        }

        bool check_mission = false;
        int16_t count = 0;
        int16_t index_pick = -1;
        int16_t index_put = -1;

        if(root_recv["mission_type"].isBool() && root_recv["mission_code"].isInt() && root_recv["mission_details"].isArray())
        {
            bool check_action = true;
            for(const auto &item : root_recv["mission_details"])
            {
                if (item["action_name"].isString())
                {
                    /* code */
                    if(item["action_name"].asString() == "pick" || item["action_name"].asString() == "put" || item["action_name"].asString() == "navigation")
                    {
                        if(item["action_name"].asString() == "pick")
                        {
                            if(index_pick == -1)
                            {
                                index_pick = count;
                            }
                            if(item["qr_code"].isInt() && item["point_out1"].isString() && item["point_in"].isString() && item["point_out2"].isString())
                            {
                                std::cout << "--------ok pick\n";
                                continue;
                            }
                            else
                            {
                                std::cout << "--------error pick\n";
                            }
                        }
                        else if(item["action_name"].asString() == "put")
                        {
                            if(index_put == -1)
                            {
                                index_put = count;
                            }
                            if(item["point_out1"].isString() && item["point_in"].isString() && item["point_out2"].isString())
                            {
                                std::cout << "-------ok put\n";
                                continue;
                            }
                            else
                            {
                                std::cout << "-------error put\n";
                            }
                        }
                        else if(item["action_name"].asString() == "navigation")
                        {
                            if(item["target_point"].isString())
                            {
                                continue;
                            }
                            else
                            {
                                std::cout << "--------error nav\n";
                            }
                        }
                    }
                }
                std::cout << "--------error " << item["action_name"].asString() << "\n";
                check_action = false;
                count++;
            }
            if(check_action)
            {
                
                check_mission = true;
            }
        }

        COMMODITY_STATE comidity;
        LIFT_LEVEL lift_state;
        {
            std::lock_guard<std::mutex> lock(mutex_amr_hardware);
            comidity = amr_hardware.comidity_state;
            lift_state = amr_hardware.lift_state;
        }
        
        if(comidity == COMMODITY_AVAI)
        {
            std::cout << "commodity available----------\n";
            if((index_put == -1 && index_pick >= 0) || (index_pick >= 0 && index_put >= 0 && index_pick < index_put))
            {
                check_mission = false;
            }
        }

        if(!check_mission)
        {
            res.status = 400;
            res.set_content("{\"error\":\"missing or invalid key\"}", "application/json");
            return;
        }

        std::cout << "robot state : " << (int)robot_state << std::endl;

        Json::StreamWriterBuilder writer;
        MISSION_CONFIRM_TYPE mission_confirm;
        Json::Value reply;
        {
            std::lock_guard<std::mutex> lock(lock_mission_response_save);
            mission_confirm = mission_response_save.mission_confirm;
        }
        if(robot_state == ROBOT_STATE_IDLE && mission_confirm != MISSION_CONFIRM_RUN && root_recv["mission_type"].asBool() == amr_work_mode && lift_state == LIFT_LEVEL_0)
        {
            std::string mission_details_string = Json::writeString(writer,root_recv["mission_details"]);
            Mission_Request_Structure mission_req;
            mission_req.mission_code = root_recv["mission_code"].asInt();
            memset(mission_req.mission_details,0,MISSION_BUFFER_SIZE);
            memcpy(mission_req.mission_details,mission_details_string.c_str(),mission_details_string.size());
            
            if(mq_send(mq_mission_req,reinterpret_cast<char*>(&mission_req),sizeof(Mission_Request_Structure),0) == -1)
            {
                if (errno == EAGAIN) 
                {
                    // queue full → drop hoặc overwrite
                }
                reply["response"] = 1;
            }
            else
            {
                reply["response"] = 0;
            }
        }
        else
        {
            reply["response"] = 1;
        }

        res.status = 200;
        res.set_content(Json::writeString(writer, reply), "application/json");
    });

    http_server.Get("/status", [](const httplib::Request& req, httplib::Response& res) {
        Json::Value root;
        {
            std::lock_guard<std::mutex> lock(mutex_seer_state);
            if(amr_seer_state["valid"].asBool())
            {
                root["seer"] = amr_seer_state["state"];
            }
            else
            {
                root["seer"] = Json::nullValue;
            }
        }

        {
            Json::Value mission_active_json;
            std::lock_guard<std::mutex> lock(mutex_mission_active);
            mission_active_json["action_index"] = mission_active.action_index;
            mission_active_json["mission_code"] = mission_active.mission_code;
            mission_active_json["mission_error_code"] = mission_active.mission_error_code;
            mission_active_json["mission_state"] = mission_active.mission_state;
            root["mission"] = mission_active_json;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_amr_hardware);
            root["lift"] = amr_hardware.lift_state;
            root["commodity"] = amr_hardware.comidity_state;
        }
        root["robot_state"] = robot_state;

        root["work_mode"] = (bool)amr_work_mode;

        Json::StreamWriterBuilder writer;
        res.set_content(Json::writeString(writer, root), "application/json");
    });
}

int main(int argc,char *argv[])
{
    bool check_mq = false;

    amr_seer_state["seer_state"] = Json::nullValue;
    amr_seer_state["valid"] = false;

    mission_active.action_index = -1;
    mission_active.mission_code = -1;
    mission_active.mission_error_code = MISSION_ERROR_NONE;
    mission_active.mission_state = MISSION_STATE_NONE;

    amr_hardware.comidity_state = COMMODITY_NONE;
    amr_hardware.determine_point = "";
    amr_hardware.lift_state = LIFT_NONE;

    signal(SIGINT, terminal_sig_callback);
    signal(SIGTERM, terminal_sig_callback);

    struct mq_attr attr{};
    attr.mq_flags   = 0;                  // blocking receive
    attr.mq_maxmsg  = 1;                  // queue size = 1
    attr.mq_msgsize = sizeof(Mission_Response_Structure);// message size
    attr.mq_curmsgs = 0;
    mq_mission_res = mq_open(MQ_MISSION_RES,O_CREAT | O_RDONLY,0666,&attr);
    if (mq_mission_res == (mqd_t)-1) 
    {
        perror("consumer mq mission response open");
        return 1;
    }
    std::cout << "Message Queue for Mission Res is created\n";

    /*---------------------------------------------------------*/
    while (running) 
    {
        mq_mission_req = mq_open(MQ_MISSION_REQ, O_WRONLY | O_NONBLOCK);
        if (mq_mission_req != (mqd_t)-1) 
        {
            check_mq = true;
            break;
        }
        if (errno == ENOENT) 
        {
            std::cout << "[Producer] MQ not ready, retry...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));; // 200 ms
        } 
        else 
        {
            perror("producer mq mission request open");
            return 1;
        }
    }
    if (!check_mq)
    {
        /* code */
        return -1;
    }
    std::cout << "Connected to Message Queue Mission Req\n";
    /*---------------------------------------------------------*/

    /*---------------------------------------------------------*/
    check_mq = false;
    while (running) 
    {
        mq_signal_control = mq_open(MQ_SIGNAL_CTRL, O_WRONLY | O_NONBLOCK);
        if (mq_signal_control != (mqd_t)-1) 
        {
            check_mq = true;
            break;
        }
        if (errno == ENOENT) 
        {
            std::cout << "[Producer] MQ not ready, retry...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));; // 200 ms
        } 
        else 
        {
            perror("producer mq signal control open");
            return 1;
        }
    }
    if (!check_mq)
    {
        /* code */
        return -1;
    }
    std::cout << "Connected to Message Queue Signal Control\n";
    /*---------------------------------------------------------*/
    
    /*--------Shared memory----------*/
    if (!wait_and_attach_amr_state())
    {
        /* code */
        return 1;
    }
    /*-------------------------------*/

    http_server_init();

    std::thread thread_read_state_amr(thread_read_state_amr_func);
    thread_read_state_amr.detach();

    http_server.new_task_queue = [] { return new httplib::ThreadPool(4); };
    std::thread http_server_thread([&]() {
        std::cout << "HTTP server starting on port 9000...\n";
        http_server.listen("0.0.0.0", 9000);  // BLOCKING
        std::cout << "HTTP server stopped.\n";
    });

    while (running)
    {
        /* code */
        Mission_Response_Structure mission_res = {0};
        unsigned int prio;
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;
        ssize_t n = mq_timedreceive(mq_mission_res,reinterpret_cast<char*>(&mission_res),sizeof(Mission_Response_Structure),&prio,&ts);
        if (n >= 0) 
        {
            std::cout << "mission code : " << mission_res.mission_code << "\tmission confirm type : " << mission_res.mission_confirm << std::endl;
            {
                std::lock_guard<std::mutex> lock(lock_mission_response_save);
                mission_response_save.mission_code = mission_res.mission_code;
                mission_response_save.mission_confirm = mission_res.mission_confirm;
            }
        }
        else
        {
            if (errno == ETIMEDOUT) 
            {
                continue;   // check running again
            }
            if (errno == EINTR) 
            {
                continue;   // signal interrupt
            }
            perror("mq_timedreceive");
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    http_server.stop(); 
    http_server_thread.join(); 

    mq_close(mq_mission_res);
    mq_unlink(MQ_MISSION_RES);

    cleanup();

    return 0;
}