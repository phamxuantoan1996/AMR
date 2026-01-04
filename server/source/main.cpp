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

std::atomic<bool> running{true};
httplib::Server http_server;

mqd_t mq_mission_req;
mqd_t mq_mission_res;
mqd_t mq_signal_control;

Mission_Response_Structure mission_response_save;
std::mutex lock_mission_response_save;


void terminal_sig_callback(int) 
{ 
    running = false; 

}

void http_server_init()
{
    /* init http server */
    // POST /cancel  --> receive JSON, return JSON
    http_server.Post("/cancel", [](const httplib::Request& req, httplib::Response& res) {
        std::cout << "send cancel\n";
        
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
        
        if(!root_recv["level"].isBool())
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
        
        if(!root_recv["x"].isDouble() || !root_recv["y"].isDouble() || !root_recv["angle"].isDouble() || !root_recv["length"].isDouble())
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
    http_server.Post("/dispatch_mission", [](const httplib::Request& req, httplib::Response& res) {
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

        if(!root_recv["mission_code"].isInt() || !root_recv["mission_type"].isBool() || !root_recv["mission_details"].isArray())
        {
            res.status = 400;
            res.set_content("{\"error\":\"Key invalid!\"}", "application/json");
            return;
        }

        check = true;
        for(auto item : root_recv["mission_details"])
        {
            if(item["action_name"].isString())
            {
                if(item["action_name"].asString() == "pick")
                {
                    if (item["point_in"].isString() && item["point_out1"].isString() && item["point_out2"].isString() && item["qr_code"].isInt())
                    {
                        /* code */
                        continue;
                    }
                    
                }
                else if(item["action_name"].asString() == "put")
                {
                    if (item["point_in"].isString() && item["point_out1"].isString() && item["point_out2"].isString())
                    {
                        /* code */
                        continue;
                    }
                }
                else if(item["action_name"].asString() == "navigation")
                {
                    if (item["target_point"].isString())
                    {
                        /* code */
                        continue;
                    }
                }
            }
            check = false;
        }

        Json::StreamWriterBuilder writer;
        Json::Value reply;
        std::lock_guard<std::mutex> lock(lock_mission_response_save);
        if (check && mission_response_save.mission_confirm != MISSION_CONFIRM_RUN)
        {
            /* code */
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
}

int main(int argc,char *argv[])
{
    bool check_mq = false;
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
    

    http_server_init();

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

    return 0;
}