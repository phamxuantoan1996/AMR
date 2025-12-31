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
        Json::Value root;
        std::string errs;

        if (!Json::parseFromStream(rbuilder, ss, &root, &errs)) {
            res.status = 400;
            res.set_content("{\"error\":\"Invalid JSON\"}", "application/json");
            return;
        }
        
        
        Json::Value reply;
        reply["response"] = 0;
        Json::StreamWriterBuilder writer;
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