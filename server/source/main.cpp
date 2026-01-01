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

#include <common.h>

std::atomic<bool> running{true};
httplib::Server http_server;

void terminal_sig_callback(int) 
{ 
    running = false; 

}

void http_server_init()
{
    /* init http server */
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
        if (check)
        {
            /* code */
            std::string mission_details_string = Json::writeString(writer,root_recv["mission_details"]);
            // Mission_Request_Structure mission_req;
            // mission_req.mission_code = root_recv["mission_code"].asInt();
            // memset(mission_req.mission_details,0,MISSION_BUFFER_SIZE);
            // memcpy(mission_req.mission_details,mission_details_string.c_str(),mission_details_string.size());
            // std::cout << mission_req.mission_details << std::endl;
            reply["response"] = 0;
        }
        else
        {
            reply["response"] = 1;
        }
        res.status = 201;
        res.set_content(Json::writeString(writer, reply), "application/json");
    });
}

int main(int argc,char *argv[])
{
    signal(SIGINT, terminal_sig_callback);
    signal(SIGTERM, terminal_sig_callback);

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
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    http_server.stop(); 
    http_server_thread.join(); 

    return 0;
}