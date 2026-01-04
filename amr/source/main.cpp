#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <cerrno>
#include <signal.h>
#include <cstring>

#include <mqueue.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <sys/mman.h>

#include <jsoncpp/json/json.h>
#include <sstream>

#include <common.h>

std::atomic<bool> running{true};

mqd_t mq_mission_req;
mqd_t mq_mission_res;
mqd_t mq_signal_control;

Mission_Control_Signal_Structure mission_signal_control;
std::mutex mutex_mission_signal;

AMR_Hardware_Info_Structure amr_hardware;
std::mutex mutex_amr_hardware;

Mission_Active_Info_Structure mission_active = {0};
std::mutex mutex_mission_active;

SharedBlockAMRState* shm_amr_state = nullptr;
int shm_amr_state_fd = -1;

Json::Value seer_state_json;
std::mutex mutex_seer_state;

ROBOT_STATE robot_state = ROBOT_STATE_NONE;


std::string SeerStr = R"(
                        {"DI":[{"id":0,"source":"normal","status":false,"valid":true},{"id":1,"source":"normal","status":false,"valid":true},{"id":2,"source":"normal","status":true,"valid":true},{"id":3,"source":"normal","status":true,"valid":true},{"id":5,"source":"normal","status":false,"valid":true},{"id":6,"source":"normal","status":false,"valid":true},{"id":7,"source":"normal","status":false,"valid":true},{"id":8,"source":"normal","status":false,"valid":true},{"id":9,"source":"normal","status":false,"valid":true},{"id":10,"source":"virtual","status":false,"valid":true}],
                        "angle":0.52639999999999998,
                        "area_ids":[1,2,3],
                        "battery_level":0.70999999999999996,
                        "block_reason":1,
                        "blocked":true,
                        "charging":false,
                        "confidence":0.59330000000000005,
                        "create_on":"2025-11-06T15:24:11.589Z",
                        "current_map":"aizobo",
                        "current_station":"CP4",
                        "emergency":false,
                        "errors":[{"52200":1762413837,"desc":"robot is blocked","times":1}],
                        "last_station":"CP4",
                        "reloc_status":1,
                        "ret_code":0,
                        "target_dist":3.4793868718092797,
                        "target_id":"LM7",
                        "task_status":2,
                        "unfinished_path":["LM7","LM8","LM5"],
                        "voltage":49.100000000000001,
                        "vx":-0.0001,
                        "vy":-0,
                        "x":-11.1654,
                        "y":-0.16689999999999999}
                    )";


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

void thread_exec_mission_func()
{
    while (running)
    {
        /* code */
        Mission_Request_Structure mission_req = {0};
        unsigned int prio;
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;
        ssize_t n = mq_timedreceive(mq_mission_req,reinterpret_cast<char*>(&mission_req),sizeof(Mission_Request_Structure),&prio,&ts);
        if (n >= 0) 
        {
            std::istringstream ss(std::string(mission_req.mission_details));
            Json::CharReaderBuilder rbuilder;
            Json::Value root_mission;
            std::string errs;
            bool check = Json::parseFromStream(rbuilder, ss, &root_mission, &errs);
            if(check && root_mission.isArray())
            {
                Mission_Response_Structure mission_res = {0};
                mission_res.mission_code = mission_req.mission_code;
                mission_res.mission_confirm = MISSION_CONFIRM_RUN;
                if(mq_send(mq_mission_res,reinterpret_cast<char*>(&mission_res),sizeof(Mission_Response_Structure),0) == -1)
                {
                    
                }
                else
                {
                    
                }
                
                for(auto item : root_mission)
                {
                    if (item["action_name"].isString())
                    {
                        /* code */
                        std::cout << item["action_name"].asString() << "\n";
                        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                    }
                }
                std::cout << "mission complete\n";
                mission_res.mission_confirm = MISSION_CONFIRM_COMPLETE;
                if(mq_send(mq_mission_res,reinterpret_cast<char*>(&mission_res),sizeof(Mission_Response_Structure),0) == -1)
                {
                    
                }
                else
                {
                    
                }

            }
            else
            {
                Mission_Response_Structure mission_res = {0};
                mission_res.mission_code = mission_req.mission_code;
                mission_res.mission_confirm = MISSION_CONFIRM_NONE;

                if(mq_send(mq_mission_res,reinterpret_cast<char*>(&mission_res),sizeof(Mission_Response_Structure),0) == -1)
                {
                    
                }
                else
                {
                    
                }
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
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void thread_exec_signal_func()
{
    while (running)
    {
        /* code */
        AMR_Signal_Control_Structure amr_signal_control = {0};
        unsigned int prio;
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;
        ssize_t n = mq_timedreceive(mq_signal_control,reinterpret_cast<char*>(&amr_signal_control),sizeof(AMR_Signal_Control_Structure),&prio,&ts);
        if (n >= 0) 
        {
            if(amr_signal_control.signal_mission.signal_cancel)
            {
                std::cout << "signal cancel\n";
            }
            else if (amr_signal_control.signal_mission.signal_pause_manual)
            {
                /* code */
                std::cout << "signal pause manual\n";
            }
            else if (amr_signal_control.signal_mission.signal_resume_manual)
            {
                /* code */
                std::cout << "signal resume manual\n";
            }
            else if (amr_signal_control.signal_relocation.valid)
            {
                /* code */
                std::cout << "signal relocation\n";
            }
            else if (amr_signal_control.signal_lift != LIFT_NONE)
            {
                /* code */
                std::cout << "signal lift\n";
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
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
}

void thread_update_amr_state_func()
{
    while (true)
    {
        /* code */
        AMR_State_Structure p;
        if(seer_state_json["blocked"].asBool() || seer_state_json["emergency"].asBool())
        {
            robot_state = ROBOT_STATE_ERROR;
        }
        else if(mission_active.mission_state == MISSION_STATE_RUNNING)
        {
            robot_state = ROBOT_STATE_ACTIVE;
        }
        else if(mission_active.mission_state == MISSION_STATE_SUSPEND)
        {
            robot_state = ROBOT_STATE_STOP;
        }
        else
        {
            robot_state = ROBOT_STATE_IDLE;
        }

        memset(p.seer_state,0,SEER_STATE_SIZE);
        {
            std::lock_guard<std::mutex> lock(mutex_seer_state);
            std::string SeerStr = "";
            Json::StreamWriterBuilder writer;
            writer["indentation"] = "";
            SeerStr = Json::writeString(writer, seer_state_json);
            memcpy(p.seer_state,SeerStr.data(),SeerStr.size());
            if(!seer_state_json["current_station"].asString().empty())
            {
                amr_hardware.determine_point = seer_state_json["current_station"].asString();
            }
        }
        p.lift_state = amr_hardware.lift_state;


        Json::Value di3 = seer_state_json["DI"][3];
        // // std::cout << "id : " << di3["id"].asInt() << "\n";
        if(di3["valid"].asBool())
        {
            if(di3["status"].asBool()) 
            {
                amr_hardware.comidity_state = COMMODITY_AVAI;
            }
            else
            {
                amr_hardware.comidity_state = COMMODITY_UNAVAI;
            }
        }
        p.commodity_state = amr_hardware.comidity_state;
        
        

        p.robot_state = robot_state;
        p.mission_active_info.action_index = mission_active.action_index;
        p.mission_active_info.mission_code = mission_active.mission_code;
        p.mission_active_info.mission_error_code = mission_active.mission_error_code;
        p.mission_active_info.mission_state = mission_active.mission_state;

        // std::cout << "robot state : " << (int)robot_state << std::endl;
        int r = pthread_mutex_lock(&shm_amr_state->rw_mutex);
        if (r == EOWNERDEAD) 
        {
            std::cerr << "shm amr state EOWNERDEAD on rw_mutex, recovering...\n";
            pthread_mutex_consistent(&shm_amr_state->rw_mutex);
        } 
        else if (r != 0) 
        {
            errno = r; 
            perror("pthread_mutex_lock rw in loop");
            break;
        }
        // update state of amr for server.
        memcpy(shm_amr_state->data,&p,sizeof(AMR_State_Structure));

        if (pthread_mutex_unlock(&shm_amr_state->rw_mutex) != 0) 
        { 
            perror("pthread_mutex_unlock rw in loop"); 
            break; 
        }
        // std::cout << "1\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc,char *argv[])
{
    mission_active.action_index = -1;
    mission_active.mission_code = -1;
    mission_active.mission_error_code = MISSION_ERROR_NONE;
    mission_active.mission_state = MISSION_STATE_NONE;

    amr_hardware.comidity_state = COMMODITY_NONE;
    amr_hardware.determine_point = "";
    amr_hardware.lift_state = LIFT_NONE;

    bool check_mq = false;
    signal(SIGINT, terminal_sig_callback);
    signal(SIGTERM, terminal_sig_callback);

    /*---------------------------------------------------------*/
    struct mq_attr attr{};
    attr.mq_flags   = 0;                  // blocking receive
    attr.mq_maxmsg  = 1;                  // queue size = 1
    attr.mq_msgsize = sizeof(Mission_Request_Structure);// message size
    attr.mq_curmsgs = 0;

    mq_mission_req = mq_open(MQ_MISSION_REQ,O_CREAT | O_RDONLY,0666,&attr);
    if (mq_mission_req == (mqd_t)-1) 
    {
        perror("consumer mq request mission open");
        return 1;
    }
    std::cout << "Message Queue for Mission Req is created\n";
    /*---------------------------------------------------------*/
    attr.mq_flags   = 0;                  // blocking receive
    attr.mq_maxmsg  = 1;                  // queue size = 1
    attr.mq_msgsize = sizeof(AMR_Signal_Control_Structure);// message size
    attr.mq_curmsgs = 0;

    mq_signal_control = mq_open(MQ_SIGNAL_CTRL,O_CREAT | O_RDONLY,0666,&attr);
    if (mq_signal_control == (mqd_t)-1) 
    {
        perror("consumer mq signal control open");
        return 1;
    }
    std::cout << "Message Queue for Signal Control is created\n";
    /*---------------------------------------------------------*/

    /*---------------------------------------------------------*/
    while (running) 
    {
        mq_mission_res = mq_open(MQ_MISSION_RES, O_WRONLY | O_NONBLOCK);
        if (mq_mission_res != (mqd_t)-1) 
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
            perror("producer mq mission response open");
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



    /*------------shared memory for amr state------------------*/
    // state
    // open/create
    shm_amr_state_fd = shm_open(SHM_AMR_STATE_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_amr_state_fd < 0) 
    { 
        perror("shm_amr_state open"); 
        return 1; 
    }
    // ensure size
    if (ftruncate(shm_amr_state_fd, sizeof(SharedBlockAMRState)) != 0) 
    { 
        perror("ftruncate"); 
        close(shm_amr_state_fd); 
        return 1; 
    }
    shm_amr_state = (SharedBlockAMRState*)mmap(nullptr, sizeof(SharedBlockAMRState), PROT_READ | PROT_WRITE, MAP_SHARED, shm_amr_state_fd, 0);
    if (shm_amr_state == MAP_FAILED) 
    { 
        perror("mmap"); 
        close(shm_amr_state_fd); 
        return 1; 
    }
    bool need_amr_state_init = (shm_amr_state->magic != SHM_AMR_STATE_MAGIC);
    if (need_amr_state_init) 
    {
        // initialize memory
        memset(shm_amr_state, 0, sizeof(SharedBlockAMRState));
        // init mutex attributes    
        pthread_mutexattr_t mattr;
        if (pthread_mutexattr_init(&mattr) != 0) 
        { 
            perror("pthread_mutexattr_init"); 
            cleanup(); 
            return 1; 
        }
        if (pthread_mutexattr_setpshared(&mattr, PTHREAD_PROCESS_SHARED) != 0) 
        { 
            perror("setpshared"); 
            cleanup(); 
            return 1; 
        }
        if (pthread_mutexattr_setrobust(&mattr, PTHREAD_MUTEX_ROBUST) != 0) 
        { 
            perror("setrobust"); 
            cleanup(); 
            return 1; 
        }

        if (pthread_mutex_init(&shm_amr_state->rw_mutex, &mattr) != 0) 
        { 
            perror("pthread_mutex_init rw"); 
            cleanup(); 
            return 1; 
        }
        if (pthread_mutex_init(&shm_amr_state->rc_mutex, &mattr) != 0) 
        { 
            perror("pthread_mutex_init rc"); 
            cleanup(); 
            return 1; 
        }
        pthread_mutexattr_destroy(&mattr);
        shm_amr_state->version = 1;
        shm_amr_state->read_count = 0;
        shm_amr_state->len = 0;
        shm_amr_state->magic = SHM_AMR_STATE_MAGIC;
        std::cout << "created and initialized SHM AMR STATE (version=1)\n";
    }
    else
    {
        // attach: bump version while holding rw_mutex (handle EOWNERDEAD)
        int r = pthread_mutex_lock(&shm_amr_state->rw_mutex);
        if (r == EOWNERDEAD) {
            std::cerr << "detected previous owner died while holding rw_mutex; making consistent\n";
            // Recovery: mark consistent
            pthread_mutex_consistent(&shm_amr_state->rw_mutex);
        } 
        else if (r != 0) 
        {
            errno = r; 
            perror("pthread_mutex_lock rw");
            cleanup(); 
            return 1;
        }
        // safe to bump version and reset len
        shm_amr_state->version = shm_amr_state->version + 1;
        shm_amr_state->len = 0;
        // if desired, reset other transient fields; keep read_count (should be 0)
        if (pthread_mutex_unlock(&shm_amr_state->rw_mutex) != 0) 
        { 
            perror("pthread_mutex_unlock rw"); 
            cleanup(); 
            return 1; 
        }
        std::cout << "attached to existing SHM, bumped version = " << shm_amr_state->version << "\n";        
    }
    /*---------------------------------------------------------*/

    std::thread thread_update_state_amr(thread_update_amr_state_func);
    thread_update_state_amr.detach();

    std::thread thread_exec_mission(thread_exec_mission_func);
    thread_exec_mission.detach();

    std::thread thread_exec_signal(thread_exec_signal_func);
    thread_exec_signal.detach();

    while (running)
    {
        std::istringstream ss(SeerStr);
        Json::CharReaderBuilder rbuilder;
        Json::Value root;
        std::string errs;
        bool check_json = Json::parseFromStream(rbuilder, ss, &root, &errs);

        if(root.isObject() && check_json)
        {
            std::lock_guard<std::mutex> lock(mutex_seer_state);
            // std::cout << "current station : " << root["current_station"].asString() << "\n";
            seer_state_json = root;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    mq_close(mq_mission_req);
    mq_unlink(MQ_MISSION_REQ);
    mq_close(mq_signal_control);
    mq_unlink(MQ_SIGNAL_CTRL);

    cleanup();
    return 0;
}