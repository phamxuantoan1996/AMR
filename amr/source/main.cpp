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
#include <httplib.h>

#include <seercontroller.h>

#define AMR_CONTROL_LIFT_PAUSE 1
#define AMR_CONTROL_LIFT_UP 2
#define AMR_CONTROL_LIFT_DOWN 3

std::atomic<bool> running{true};

mqd_t mq_mission_req;
mqd_t mq_mission_res;
mqd_t mq_signal_control;

Mission_Control_Signal_Structure mission_signal_control;

AMR_Hardware_Info_Structure amr_hardware;
std::mutex mutex_amr_hardware;

Mission_Active_Info_Structure mission_active = {0};
std::mutex mutex_mission_active;

SharedBlockAMRState* shm_amr_state = nullptr;
int shm_amr_state_fd = -1;

Json::Value seer_state_json;
std::mutex mutex_seer_state;

ROBOT_STATE robot_state = ROBOT_STATE_NONE;

uint16_t board_state[10]; //input register
std::mutex mutex_board_state;

SeerController robot("192.168.192.5");

std::string point_out2; // save point_out2 of mission when amr go into rack
/*----------------------------Function---------------------------*/

std::list<std::string> gen_path(std::string source,std::string target)
{
    std::list<std::string> paths;
    httplib::Client cli("http://127.0.0.1:7000");
    Json::Value root;
    root["source"] = source;
    root["target"] = target;

    Json::StreamWriterBuilder writer;
    std::string body = Json::writeString(writer, root);

    auto res = cli.Post("/gen_path", body, "application/json");

    if(res->status == 200)
    {
        Json::CharReaderBuilder reader;
        Json::Value response;
        std::string errs;
        std::istringstream ss(res->body);
        bool ok = Json::parseFromStream(reader, ss, &response, &errs);
        if(ok)
        {
            if(response["status"].asString() == "ok")
            {
                if(response["gen_path"].isArray())
                {
                    for(auto& item : response["gen_path"])
                    {
                        paths.push_back(item.asString());
                    }
                }
            }
        }
    }
    return paths;
}

bool checkContainArea(uint16_t area)
{
    if(seer_state_json["area_ids"].isArray())
    {
        for(auto& item : seer_state_json["area_ids"])
        {
            if(item.asString() == std::to_string(area))
            {
                return true;
            }
        }
    }
    return false;
}

bool setLedRGB(uint8_t color)
{
    httplib::Client cli("http://127.0.0.1:9001");
    // Tạo JSON
    Json::Value root;
    root["start_address"] = 0;
    root["num_of_reg"] = 1;
    Json::Value value(Json::arrayValue);
    value.append(color);
    root["value"] = value;

    Json::StreamWriterBuilder writer;
    std::string body = Json::writeString(writer, root);

    // Gửi POST
    auto res = cli.Post("/holding_register", body, "application/json");

    if(res->status == 200)
    {
        Json::CharReaderBuilder reader;
        Json::Value response;
        std::string errs;
        std::istringstream ss(res->body);
        bool ok = Json::parseFromStream(reader, ss, &response, &errs);
        if(ok)
        {
            if(response["response"].asBool())
                return true;
        }
    }
    return false;
}

bool setControlBoard(uint8_t mission)
{
    httplib::Client cli("http://127.0.0.1:9001");
    // Tạo JSON
    Json::Value root;
    root["start_address"] = 1;
    root["num_of_reg"] = 1;
    Json::Value value(Json::arrayValue);
    value.append(mission);
    root["value"] = value;

    Json::StreamWriterBuilder writer;
    std::string body = Json::writeString(writer, root);

    // Gửi POST
    auto res = cli.Post("/holding_register", body, "application/json");

    if(res->status == 200)
    {
        Json::CharReaderBuilder reader;
        Json::Value response;
        std::string errs;
        std::istringstream ss(res->body);
        bool ok = Json::parseFromStream(reader, ss, &response, &errs);
        if(ok)
        {
            if(response["response"].asBool())
            return true;
        }
    }
    return false;
}

bool clearErrorBoard()
{
    httplib::Client cli("http://127.0.0.1:9001");
    // Tạo JSON
    Json::Value root;
    root["start_address"] = 7;
    root["num_of_reg"] = 1;
    Json::Value value(Json::arrayValue);
    value.append(1);
    root["value"] = value;

    Json::StreamWriterBuilder writer;
    std::string body = Json::writeString(writer, root);

    // Gửi POST
    auto res = cli.Post("/holding_register", body, "application/json");

    if(res->status == 200)
    {
        Json::CharReaderBuilder reader;
        Json::Value response;
        std::string errs;
        std::istringstream ss(res->body);
        bool ok = Json::parseFromStream(reader, ss, &response, &errs);
        if(ok)
        {
            if(response["response"].asBool())
                return true;
        }
    }
    return false;
}

void readStateBoard()
{
    httplib::Client cli("http://127.0.0.1:9001");
    // Gửi GET
    auto res = cli.Get("/input_register");

    if(res->status == 200)
    {
        Json::CharReaderBuilder reader;
        Json::Value root;
        std::string errs;
        std::istringstream ss(res->body);
        // std::cout << res->body << "\n";
        bool ok = Json::parseFromStream(reader, ss, &root, &errs);
        if(ok)
        {
            if(root["valid"].asBool() && root["value"].isArray())
            {
                LIFT_LEVEL lift_state;
                Json::Value value = root["value"];
                {
                    std::lock_guard<std::mutex> lock(mutex_board_state);
                    for(uint8_t i = 0; i < 10; i++)
                    {
                        board_state[i] = value[i].asInt();
                    }
                    if(board_state[5] == 1 && board_state[6] == 0)
                    {
                        lift_state = LIFT_LEVEL_1;
                    } 
                    else if(board_state[5] == 0 && board_state[6] == 1)
                    {
                        lift_state = LIFT_LEVEL_0;
                    }
                    else
                    {
                        lift_state = LIFT_NONE;
                    }
                }

                {
                    std::lock_guard<std::mutex> lock(mutex_amr_hardware);
                    amr_hardware.lift_state = lift_state;
                }
            }
        }
    }
}

bool requestScanDataMatrix()
{
    httplib::Client cli("http://127.0.0.1:9002");
    // Tạo JSON
    Json::Value root;
    root["capture"] = true;

    Json::StreamWriterBuilder writer;
    std::string body = Json::writeString(writer, root);

    // Gửi POST
    auto res = cli.Post("/data_matrix", body, "application/json");

    if(res->status == 200)
    {
        Json::CharReaderBuilder reader;
        Json::Value response;
        std::string errs;
        std::istringstream ss(res->body);
        bool ok = Json::parseFromStream(reader, ss, &response, &errs);
        if(ok)
        {
            if(response["response"].asBool())
                return true;
        }
    }
    return false;
}

bool getQrCode(uint32_t &number)
{
    httplib::Client cli("http://127.0.0.1:9002");

    // Gửi GET
    auto res = cli.Get("/data_matrix");

    if(res->status == 200)
    {
        Json::CharReaderBuilder reader;
        Json::Value response;
        std::string errs;
        std::istringstream ss(res->body);
        bool ok = Json::parseFromStream(reader, ss, &response, &errs);
        if(ok)
        {
            if(response["valid"].asBool())
            {
                number = response["value"].asInt();
                return true;
            }
        }
    }
    return false;
}

bool checkTarget(std::string point)
{
    
    if(seer_state_json["task_status"].asInt() == 4 && seer_state_json["current_station"].asString() == point && seer_state_json["unfinished_path"].size() == 0)
    {
        return true;
    }
    return false;
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

                std::list<Json::Value> task_list;
                int32_t action_index = 0;

                for(const auto &item : root_mission)
                {
                    if(item["action_name"].asString() == "pick")
                    {
                        if(item["qr_code"].asInt() > 0)
                        {
                            Json::Value task_nav_scan;
                            task_nav_scan["task_name"] = "navigation_scan";
                            task_nav_scan["target_point"] = item["point_in"];
                            task_nav_scan["action_index"] = action_index;
                            task_nav_scan["point_out2"] = item["point_out2"];

                            /*task scan*/
                            Json::Value task_scan;
                            task_scan["task_name"] = "scan";
                            task_scan["qr_code"] = item["qr_code"];
                            task_scan["point_in"] = item["point_in"];
                            /*--------*/

                            task_list.push_back(task_nav_scan);
                            task_list.push_back(task_scan);
                        }
                        else
                        {
                            Json::Value task_nav;
                            task_nav["task_name"] = "navigation";
                            task_nav["target_point"] = item["point_in"];
                            task_nav["action_index"] = action_index;
                            task_nav["point_out2"] = item["point_out2"];
                            task_list.push_back(task_nav);
                        }
                        
                        Json::Value task_pick;
                        task_pick["task_name"] = "pick";
                        task_pick["action_index"] = action_index;

                        Json::Value task_nav_out2;
                        task_nav_out2["task_name"] = "navigation";
                        task_nav_out2["target_point"] = item["point_out2"];
                        task_nav_out2["action_index"] = action_index;
                        task_nav_out2["point_out2"] = item["point_out2"];

                        Json::Value task_put;
                        task_put["task_name"] = "put";
                        task_put["action_index"] = action_index;

                        task_list.push_back(task_pick);
                        task_list.push_back(task_nav_out2);
                        task_list.push_back(task_put);
                    }
                    if(item["action_name"].asString() == "put")
                    {
                        Json::Value task_nav_out1;
                        task_nav_out1["task_name"] = "navigation";
                        task_nav_out1["target_point"] = item["point_out1"];
                        task_nav_out1["action_index"] = action_index;

                        Json::Value task_pick;
                        task_pick["task_name"] = "pick";
                        task_pick["action_index"] = action_index;

                        Json::Value task_nav_in;
                        task_nav_in["task_name"] = "navigation";
                        task_nav_in["target_point"] = item["point_in"];
                        task_nav_in["action_index"] = action_index;
                        task_nav_in["point_out2"] = item["point_out2"];
                        
                        Json::Value task_put;
                        task_put["task_name"] = "put";
                        task_put["action_index"] = action_index;

                        Json::Value task_nav_out2;
                        task_nav_out2["task_name"] = "navigation";
                        task_nav_out2["target_point"] = item["point_out2"];
                        task_nav_out2["action_index"] = action_index;
                        task_nav_out2["point_out2"] = item["point_out2"];

                        task_list.push_back(task_nav_out1);
                        task_list.push_back(task_pick);
                        task_list.push_back(task_nav_in);
                        task_list.push_back(task_put);
                        task_list.push_back(task_nav_out2);
                    }
                    if(item["action_name"].asString() == "navigation")
                    {
                        Json::Value task_nav;
                        task_nav["task_name"] = "navigation";
                        task_nav["target_point"] = item["target_point"];
                        task_nav["action_index"] = action_index;
                        task_list.push_back(task_nav);
                    }
                    action_index++;
                }

                {
                    std::lock_guard<std::mutex> lock(mutex_mission_active);
                    mission_active.mission_error_code = MISSION_ERROR_NONE;
                    mission_active.mission_state = MISSION_STATE_RUNNING;
                    mission_active.mission_code = mission_req.mission_code;
                }



                for(const auto &task : task_list)
                {
                    // amr_mission_response["action_index"] = task["action_index"];
                    {
                        std::lock_guard<std::mutex> lock(mutex_mission_active);
                        mission_active.action_index = task["action_index"].asInt();
                    }
                    if(task["task_name"] == "scan")
                    {
                        int8_t number_scan = 3;
                        uint32_t qrcode = 0;
                        while (number_scan > 0)
                        {
                            while(!requestScanDataMatrix())
                            {
                                std::cout << "sending scan qrcode command\n";
                                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            while (!getQrCode(qrcode))
                            {
                                std::cout << "sending read qrcode command\n";
                                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            }
                            if (qrcode != 0)
                            {
                                /* code */
                                break;
                            }
                            number_scan--;
                        }
                        std::cout << "qrcode : " << qrcode << std::endl;
                        if (qrcode != task["qr_code"].asInt())
                        {
                            /* code */
                            robot.cancelNavigation();
                            // amr_mission_response["mission_error_code"] = MISSION_ERROR_QRCODE;
                            {
                                std::lock_guard<std::mutex> lock(mutex_mission_active);
                                mission_active.mission_error_code = MISSION_ERROR_QRCODE;
                            }
                            break;
                        }
                        else
                        {
                            std::cout << "resume nav scan --------------------\n";
                            robot.resumeNavigation();
                            while(!checkTarget(task["point_in"].asString()))
                            {
                                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            }
                        }

                    }
                    if(task["task_name"] == "pick")
                    {
                        uint16_t board_state_value1 = 0;
                        uint16_t board_state_value2 = 0;
                        while (!setControlBoard(AMR_CONTROL_LIFT_UP))
                        {
                            /* code */
                            std::cout << "sending lift up command 1\n";
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                        }
                        {
                            std::lock_guard<std::mutex> lock(mutex_board_state);
                            board_state_value1 = board_state[1];
                        }
                        while (board_state_value1 != AMR_CONTROL_LIFT_UP)
                        {
                            /* code */
                            std::cout << "sending lift up command 2\n";
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            {
                                std::lock_guard<std::mutex> lock(mutex_board_state);
                                board_state_value1 = board_state[1];
                            }
                        }

                        {
                            std::lock_guard<std::mutex> lock(mutex_board_state);
                            board_state_value1 = board_state[5];
                            board_state_value2 = board_state[7];
                        }
                        while (board_state_value1 == 0 && board_state_value2 == 0)
                        {
                            /* code */
                            std::cout << "amr picking\n";

                            // if(amr_control_signal["cancel"].asBool())
                            if(mission_signal_control.signal_cancel)
                            {
                                while (!setControlBoard(AMR_CONTROL_LIFT_PAUSE))
                                {
                                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                }
                                break;
                            }
                            // if(amr_control_signal["pause_manual"].asBool())
                            if(mission_signal_control.signal_pause_manual)
                            {
                                // amr_mission_response["mission_state"] = MISSION_STATE_SUSPEND;
                                {
                                    std::lock_guard<std::mutex> lock(mutex_mission_active);
                                    mission_active.mission_state = MISSION_STATE_SUSPEND;
                                }
                                while (!setControlBoard(AMR_CONTROL_LIFT_PAUSE))
                                {
                                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                }
                                // amr_control_signal["pause_manual"] = false;
                                mission_signal_control.signal_pause_manual = false;
                            }
                            // if(amr_control_signal["resume_manual"].asBool())
                            if(mission_signal_control.signal_resume_manual)
                            {
                                // amr_mission_response["mission_state"] = MISSION_STATE_RUNNING;
                                {
                                    std::lock_guard<std::mutex> lock(mutex_mission_active);
                                    mission_active.mission_state = MISSION_STATE_RUNNING;
                                }
                                while (!setControlBoard(AMR_CONTROL_LIFT_UP))
                                {
                                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                }
                                // amr_control_signal["resume_manual"] = false;
                                mission_signal_control.signal_resume_manual = false;
                            }

                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            {
                                std::lock_guard<std::mutex> lock(mutex_board_state);
                                board_state_value1 = board_state[5];
                                board_state_value2 = board_state[7];
                            }
                        }

                        {
                            std::lock_guard<std::mutex> lock(mutex_board_state);
                            board_state_value1 = board_state[1];
                        }
                        while (board_state_value1 == AMR_CONTROL_LIFT_UP)
                        {
                            /* code */
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            {
                                std::lock_guard<std::mutex> lock(mutex_board_state);
                                board_state_value1 = board_state[1];
                            }
                        }
                        // if(board_state[7] == 1 || amr_control_signal["cancel"].asBool())
                        {
                            std::lock_guard<std::mutex> lock(mutex_board_state);
                            board_state_value1 = board_state[7];
                        }
                        if(board_state_value1 || mission_signal_control.signal_cancel)
                        {
                            break;
                        }
                    }
                    if(task["task_name"] == "put")
                    {
                        uint16_t board_state_value1;
                        uint16_t board_state_value2;
                        while (!setControlBoard(AMR_CONTROL_LIFT_DOWN))
                        {
                            /* code */
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                        }

                        {
                            std::lock_guard<std::mutex> lock(mutex_board_state);
                            board_state_value1 = board_state[1];
                        }
                        while (board_state_value1 != AMR_CONTROL_LIFT_DOWN)
                        {
                            /* code */
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            {
                                std::lock_guard<std::mutex> lock(mutex_board_state);
                                board_state_value1 = board_state[1];
                            }
                        }

                        {
                            std::lock_guard<std::mutex> lock(mutex_board_state);
                            board_state_value1 = board_state[6];
                            board_state_value2 = board_state[7];
                        }
                        while (board_state_value1 == 0 && board_state_value2 == 0)
                        {
                            /* code */
                            std::cout << "amr putting\n";
                            // if(amr_control_signal["cancel"].asBool())
                            if(mission_signal_control.signal_cancel)
                            {
                                while (!setControlBoard(AMR_CONTROL_LIFT_PAUSE))
                                {
                                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                }
                                break;
                            }
                            // if(amr_control_signal["pause_manual"].asBool())
                            if(mission_signal_control.signal_pause_manual)
                            {
                                // amr_mission_response["mission_state"] = MISSION_STATE_SUSPEND;
                                {
                                    std::lock_guard<std::mutex> lock(mutex_mission_active);
                                    mission_active.mission_state = MISSION_STATE_SUSPEND;
                                }
                                while (!setControlBoard(AMR_CONTROL_LIFT_PAUSE))
                                {
                                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                }
                                // amr_control_signal["pause_manual"] = false;
                                mission_signal_control.signal_pause_manual = false;
                            }
                            // if(amr_control_signal["resume_manual"].asBool())
                            if(mission_signal_control.signal_resume_manual)
                            {
                                // amr_mission_response["mission_state"] = MISSION_STATE_RUNNING;
                                {
                                    std::lock_guard<std::mutex> lock(mutex_mission_active);
                                    mission_active.mission_state = MISSION_STATE_RUNNING;
                                }
                                while (!setControlBoard(AMR_CONTROL_LIFT_DOWN))
                                {
                                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                }
                                // amr_control_signal["resume_manual"] = false;
                                mission_signal_control.signal_resume_manual = false;
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            {
                                std::lock_guard<std::mutex> lock(mutex_board_state);
                                board_state_value1 = board_state[6];
                                board_state_value2 = board_state[7];
                            }
                        }

                        {
                            std::lock_guard<std::mutex> lock(mutex_board_state);
                            board_state_value1 = board_state[1];
                        }
                        while (board_state_value1 == AMR_CONTROL_LIFT_DOWN)
                        {
                            /* code */
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            {
                                std::lock_guard<std::mutex> lock(mutex_board_state);
                                board_state_value1 = board_state[1];
                            }
                        }
                        // if(board_state[7] == 1 || amr_control_signal["cancel"].asBool())
                        {
                            std::lock_guard<std::mutex> lock(mutex_board_state);
                            board_state_value1 = board_state[7];
                        }
                        if(board_state_value1 == 1 || mission_signal_control.signal_cancel)
                        {
                            break;
                        }
                    }
                    if(task["task_name"] == "navigation_scan")
                    {
                        std::cout << "amr moving to point : " << task["target_point"] << std::endl;
                        
                        robot.navigation(task["target_point"].asString());

                        bool comidity_state;
                        {
                            std::lock_guard<std::mutex> lock(mutex_amr_hardware);
                            comidity_state = amr_hardware.comidity_state;
                        }
                        // while (!checkTarget(task["target_point"].asString()) && amr_hardware_info["commodity"].asInt() != LIFT_LEVEL_1)
                        while (!checkTarget(task["target_point"].asString()) && comidity_state != LIFT_LEVEL_1)
                        {
                            /* code */
                            // if(amr_control_signal["cancel"].asBool())
                            if(mission_signal_control.signal_cancel)
                            {
                                robot.cancelNavigation();
                                break;
                            }
                            // if(amr_control_signal["resume_manual"].asBool())
                            if(mission_signal_control.signal_resume_manual)
                            {
                                // amr_control_signal["resume_manual"] = false;
                                // amr_mission_response["mission_state"] = MISSION_STATE_RUNNING;
                                mission_signal_control.signal_resume_manual = false;
                                {
                                    std::lock_guard<std::mutex> lock(mutex_mission_active);
                                    mission_active.mission_state = MISSION_STATE_RUNNING;
                                }
                                robot.resumeNavigation();
                            }
                            // if(amr_control_signal["pause_manual"].asBool())
                            if(mission_signal_control.signal_pause_manual)
                            {
                                // amr_control_signal["pause_manual"] = false;
                                // amr_mission_response["mission_state"] = MISSION_STATE_SUSPEND;
                                mission_signal_control.signal_pause_manual = false;
                                {
                                    std::lock_guard<std::mutex> lock(mutex_mission_active);
                                    mission_active.mission_state = MISSION_STATE_SUSPEND;
                                }
                                robot.pauseNavigation();
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }
                        robot.pauseNavigation();
                        std::cout << " ------------------------------moved scan point\n";
                        // if(amr_control_signal["cancel"].asBool())
                        if(mission_signal_control.signal_cancel)
                        {
                            break;
                        }
                    }
                    if(task["task_name"] == "navigation")
                    {
                        std::cout << "amr moving to point : " << task["target_point"] << std::endl;
                        
                        robot.navigation(task["target_point"].asString());

                        while (!checkTarget(task["target_point"].asString()))
                        {
                            /* code */
                            // if(amr_control_signal["cancel"].asBool())
                            if(mission_signal_control.signal_cancel)
                            {
                                robot.cancelNavigation();
                                break;
                            }
                            // if(amr_control_signal["resume_manual"].asBool())
                            if(mission_signal_control.signal_resume_manual)
                            {
                                // amr_mission_response["mission_state"] = MISSION_STATE_RUNNING;
                                // amr_control_signal["resume_manual"] = false;
                                {
                                    std::lock_guard<std::mutex> lock(mutex_mission_active);
                                    mission_active.mission_state = MISSION_STATE_RUNNING;
                                }
                                mission_signal_control.signal_resume_manual = false;
                                robot.resumeNavigation();
                            }
                            // if(amr_control_signal["pause_manual"].asBool())
                            if(mission_signal_control.signal_pause_manual)
                            {
                                // amr_mission_response["mission_state"] = MISSION_STATE_SUSPEND;
                                // amr_control_signal["pause_manual"] = false;
                                {
                                    std::lock_guard<std::mutex> lock(mutex_mission_active);
                                    mission_active.mission_state = MISSION_STATE_SUSPEND;
                                }
                                mission_signal_control.signal_pause_manual = false;
                                robot.pauseNavigation();
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        }
                        std::cout << " ------------------------------moved \n";
                        // if(amr_control_signal["cancel"].asBool())
                        if(mission_signal_control.signal_cancel)
                        {
                            break;
                        }
                    }
                }
                uint16_t board_state_value1;
                {
                    std::lock_guard<std::mutex> lock(mutex_board_state);
                    board_state_value1 = board_state[7];
                }
                // if(amr_control_signal["cancel"].asBool()) // mission is cancel
                if(mission_signal_control.signal_cancel)
                {
                    // amr_control_signal["cancel"] = false;
                    mission_signal_control.signal_cancel = false;
                    std::cout << "mission is canceled!\n";
                    // amr_mission_response["mission_state"] = MISSION_STATE_CANCEL;
                    {
                        std::lock_guard<std::mutex> lock(mutex_mission_active);
                        mission_active.mission_state = MISSION_STATE_CANCEL;
                    }
                } else if(board_state_value1 == 1)
                {
                    std::cout << "mission is canceled because lift error!\n";
                    // amr_mission_response["mission_error_code"] = MISSION_ERROR_LIFT;
                    // amr_mission_response["mission_state"] = MISSION_STATE_CANCEL;
                    {
                        std::lock_guard<std::mutex> lock(mutex_mission_active);
                        mission_active.mission_error_code = MISSION_ERROR_LIFT;
                        mission_active.mission_state = MISSION_STATE_CANCEL;
                    }
                    clearErrorBoard();
                } //else if(amr_mission_response["mission_error_code"].asInt() == MISSION_ERROR_QRCODE)
                else if(mission_active.mission_error_code == MISSION_ERROR_QRCODE)
                {
                    std::cout << "mission is canceled because scan qrcode error!\n";
                    // amr_mission_response["mission_state"] = MISSION_STATE_CANCEL;
                    {
                        std::lock_guard<std::mutex> lock(mutex_mission_active);
                        mission_active.mission_state = MISSION_STATE_CANCEL;
                    }
                } //else if(amr_mission_response["mission_error_code"].asInt() == MISSION_ERROR_PATH)
                else if(mission_active.mission_error_code == MISSION_ERROR_PATH)
                {
                    std::cout << "mission is canceled because can't generate path!\n";
                    // amr_mission_response["mission_state"] = MISSION_STATE_CANCEL;
                    {
                        std::lock_guard<std::mutex> lock(mutex_mission_active);
                        mission_active.mission_state = MISSION_STATE_CANCEL;
                    }
                }
                else // mission is completed
                {
                    // amr_mission_response["mission_state"] = MISSION_STATE_COMPLETE;
                    {
                        std::lock_guard<std::mutex> lock(mutex_mission_active);
                        mission_active.mission_state = MISSION_STATE_CANCEL;
                    }
                }

                // amr_mission_response["action_index"] = -1;
                {
                    std::lock_guard<std::mutex> lock(mutex_mission_active);
                    mission_active.action_index = -1;
                }
                robot_state = ROBOT_STATE_IDLE;
                std::cout << "-------------------------mission complete---------------------\n";
                mission_res.mission_confirm = MISSION_CONFIRM_COMPLETE;
                if(mq_send(mq_mission_res,reinterpret_cast<char*>(&mission_res),sizeof(Mission_Response_Structure),0) == -1)
                {
                    
                }
                else
                {
                    
                }
                std::cout << "-------------------------mission confirm complete---------------------\n";

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
                mission_signal_control.signal_cancel = true;
            }
            else if (amr_signal_control.signal_mission.signal_pause_manual)
            {
                /* code */
                std::cout << "signal pause manual\n";
                mission_signal_control.signal_pause_manual = true;
            }
            else if (amr_signal_control.signal_mission.signal_resume_manual)
            {
                /* code */
                std::cout << "signal resume manual\n";
                mission_signal_control.signal_resume_manual = true;
            }
            else if (amr_signal_control.signal_relocation.valid)
            {
                /* code */
                if(robot_state == ROBOT_STATE_IDLE)
                {
                    std::cout << "signal relocation\n";
                    std::cout << "amr excute relocation\n";
                    robot.relocation(amr_signal_control.signal_relocation.x,amr_signal_control.signal_relocation.y,amr_signal_control.signal_relocation.angle,amr_signal_control.signal_relocation.length);
                    while(!robot.confirmCorrectLocation())
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    }
                }
            }
            else if (amr_signal_control.signal_lift != LIFT_NONE)
            {
                /* code */
                if(robot_state == ROBOT_STATE_IDLE)
                {
                    uint16_t board_state_value1;
                    uint16_t board_state_value2;
                    {
                        std::lock_guard<std::mutex> lock(mutex_board_state);
                        board_state_value1 = board_state[5];
                        board_state_value2 = board_state[6];
                    }
                    if(amr_signal_control.signal_lift == LIFT_LEVEL_1 && board_state_value1 == 0)
                    {
                        std::cout << "lift up\n";
                        //////////////
                        while (!setControlBoard(AMR_CONTROL_LIFT_UP))
                        {
                            /* code */
                            std::cout << "sending lift up command 1\n";
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                        }
                        {
                            std::lock_guard<std::mutex> lock(mutex_board_state);
                            board_state_value1 = board_state[1];
                        }
                        while (board_state_value1 != AMR_CONTROL_LIFT_UP)
                        {
                            /* code */
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            std::cout << "sending lift up command 2\n";
                            {
                                std::lock_guard<std::mutex> lock(mutex_board_state);
                                board_state_value1 = board_state[1];
                            }
                        }
                        {
                            std::lock_guard<std::mutex> lock(mutex_board_state);
                            board_state_value1 = board_state[5];
                            board_state_value2 = board_state[7];
                        }
                        while (board_state_value1 == 0 && board_state_value2 == 0)
                        {
                            /* code */
                            std::cout << "amr uping\n";
                            {
                                bool emc;
                                {
                                    std::lock_guard<std::mutex> lock(mutex_seer_state);
                                    emc = seer_state_json["emergency"].asBool();
                                }
                                if(emc)
                                {
                                    while (!setControlBoard(AMR_CONTROL_LIFT_PAUSE))
                                    {
                                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                    }
                                    break;
                                }
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            {
                                std::lock_guard<std::mutex> lock(mutex_board_state);
                                board_state_value1 = board_state[5];
                                board_state_value2 = board_state[7];
                            }
                        }
                        {
                            std::lock_guard<std::mutex> lock(mutex_board_state);
                            board_state_value1 = board_state[1];
                        }
                        while (board_state_value1 == AMR_CONTROL_LIFT_UP)
                        {
                            /* code */
                            
                            bool emc;
                            {
                                std::lock_guard<std::mutex> lock(mutex_seer_state);
                                emc = seer_state_json["emergency"].asBool();
                            }
                            if(emc)
                            {
                                while (!setControlBoard(AMR_CONTROL_LIFT_PAUSE))
                                {
                                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                }
                                break;
                            }
                            
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            {
                                std::lock_guard<std::mutex> lock(mutex_board_state);
                                board_state_value1 = board_state[1];
                            }
                        }
                        //////////////
                    }
                    else if(amr_signal_control.signal_lift == LIFT_LEVEL_0 && board_state_value2 == 0)
                    {
                        std::cout << "lift down\n";
                        while (!setControlBoard(AMR_CONTROL_LIFT_DOWN))
                        {
                            /* code */
                            std::cout << "sending lift down command 1\n";
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                        }
                        
                        {
                            std::lock_guard<std::mutex> lock(mutex_board_state);
                            board_state_value1 = board_state[1];
                        }
                        while (board_state_value1 != AMR_CONTROL_LIFT_DOWN)
                        {
                            /* code */
                            std::cout << "sending lift down command 2\n";
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            {
                                std::lock_guard<std::mutex> lock(mutex_board_state);
                                board_state_value1 = board_state[1];
                            }
                        }

                        {
                            std::lock_guard<std::mutex> lock(mutex_board_state);
                            board_state_value1 = board_state[6];
                            board_state_value2 = board_state[7];
                        }
                        while (board_state_value1 == 0 && board_state_value2 == 0)
                        {
                            /* code */
                            std::cout << "amr downing\n";
                            
                            bool emc;
                            {
                                std::lock_guard<std::mutex> lock(mutex_seer_state);
                                emc = seer_state_json["emergency"].asBool();
                            }
                            if(emc)
                            {
                                while (!setControlBoard(AMR_CONTROL_LIFT_PAUSE))
                                {
                                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                }
                                break;
                            }
                            
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            {
                                std::lock_guard<std::mutex> lock(mutex_board_state);
                                board_state_value1 = board_state[6];
                                board_state_value2 = board_state[7];
                            }
                        }

                        {
                            std::lock_guard<std::mutex> lock(mutex_board_state);
                            board_state_value1 = board_state[1];
                        }
                        while (board_state_value1 == AMR_CONTROL_LIFT_DOWN)
                        {
                            /* code */
                            bool emc;
                            {
                                std::lock_guard<std::mutex> lock(mutex_seer_state);
                                emc = seer_state_json["emergency"].asBool();
                            }
                            if(seer_state_json["emergency"].asBool())
                            {
                                while (!setControlBoard(AMR_CONTROL_LIFT_PAUSE))
                                {
                                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                }
                                break;
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            {
                                std::lock_guard<std::mutex> lock(mutex_board_state);
                                board_state_value1 = board_state[1];
                            }
                        }
                    }
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

void thread_update_amr_state_func()
{
    while (true)
    {
        /* code */
        AMR_State_Structure p;
        MISSION_STATE mission_state;
        {
            std::lock_guard<std::mutex> lock(mutex_mission_active);
            mission_state = mission_active.mission_state;
        }
        if(seer_state_json["blocked"].asBool() || seer_state_json["emergency"].asBool())
        {
            robot_state = ROBOT_STATE_ERROR;
        }
        else if(mission_state == MISSION_STATE_RUNNING)
        {
            robot_state = ROBOT_STATE_ACTIVE;
        }
        else if(mission_state == MISSION_STATE_SUSPEND)
        {
            robot_state = ROBOT_STATE_STOP;
        }
        else
        {
            robot_state = ROBOT_STATE_IDLE;
        }

        memset(p.seer_state,0,SEER_STATE_SIZE);
        std::string determine_point = "";
        Json::Value di3;
        {
            std::lock_guard<std::mutex> lock(mutex_seer_state);
            std::string SeerStr = "";
            Json::StreamWriterBuilder writer;
            writer["indentation"] = "";
            SeerStr = Json::writeString(writer, seer_state_json);
            memcpy(p.seer_state,SeerStr.data(),SeerStr.size());
            
            determine_point = seer_state_json["current_station"].asString();
            
            di3 = seer_state_json["DI"][3];
        }
        {
            std::lock_guard<std::mutex> lock(mutex_amr_hardware);
            if(!determine_point.empty())
            {
                amr_hardware.determine_point = determine_point;
            }
            p.lift_state = amr_hardware.lift_state;
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
        }
        
        // // std::cout << "id : " << di3["id"].asInt() << "\n";
        
        p.robot_state = robot_state;
        {
            std::lock_guard<std::mutex> lock(mutex_mission_active);
            p.mission_active_info.action_index = mission_active.action_index;
            p.mission_active_info.mission_code = mission_active.mission_code;
            p.mission_active_info.mission_error_code = mission_active.mission_error_code;
            p.mission_active_info.mission_state = mission_active.mission_state;
        }
        

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
    amr_hardware.determine_point = "LM69";
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

    /*---------------Seer Init---------------------------------*/
    robot.addStatusKey("current_station");
    robot.addStatusKey("x");
    robot.addStatusKey("y");
    robot.addStatusKey("angle");
    robot.addStatusKey("confidence");
    robot.addStatusKey("last_station");
    robot.addStatusKey("blocked");
    robot.addStatusKey("block_reason");
    robot.addStatusKey("area_ids");
    robot.addStatusKey("vx");
    robot.addStatusKey("vy");
    robot.addStatusKey("w");
    robot.addStatusKey("battery_level");
    robot.addStatusKey("battery_temp");
    robot.addStatusKey("task_status");
    robot.addStatusKey("target_id");
    robot.addStatusKey("emergency");
    robot.addStatusKey("unfinished_path");
    robot.addStatusKey("target_dist");
    robot.addStatusKey("reloc_status");
    robot.addStatusKey("current_map");
    robot.addStatusKey("charging");
    robot.addStatusKey("errors");
    robot.addStatusKey("voltage");
    robot.addStatusKey("DI");
    robot.startCommunication();
    robot.confirmCorrectLocation();
    /*---------------------------------------------------------*/

    std::thread thread_update_state_amr(thread_update_amr_state_func);
    thread_update_state_amr.detach();

    std::thread thread_exec_mission(thread_exec_mission_func);
    thread_exec_mission.detach();

    std::thread thread_exec_signal(thread_exec_signal_func);
    thread_exec_signal.detach();

    robot_state = ROBOT_STATE_IDLE;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    uint8_t led_cur = 0;
    setLedRGB(10);
    led_cur = 10;

    while (running)
    {
        readStateBoard();
        robot.requestAllStatusSrc();
        {
            std::lock_guard<std::mutex> lock(mutex_seer_state);
            seer_state_json = robot.getAllStatusSrc();
        }
        // std::cout << robot_state << "\n";
        MISSION_ERROR_CODE mission_error_code;
        {
            std::lock_guard<std::mutex> lock(mutex_mission_active);
            mission_error_code = mission_active.mission_error_code;
        }
        if(mission_active.mission_error_code != MISSION_ERROR_NONE)
        {
            if(led_cur != 8)
            {
                setLedRGB(8);
                led_cur = 8;
            }
        }
        else
        {
            switch(robot_state)
            {
                case ROBOT_STATE_ERROR:
                {
                    if(led_cur != 8)
                    {
                        setLedRGB(8);
                        led_cur = 8;
                    }
                    break;
                }
                case ROBOT_STATE_IDLE:
                {
                    if(led_cur != 10)
                    {
                        setLedRGB(10);
                        led_cur = 10;
                    }
                    break;
                }
                case ROBOT_STATE_ACTIVE:
                case ROBOT_STATE_STOP:
                {
                    if(led_cur != 2)
                    {
                        setLedRGB(2);
                        led_cur = 2;
                    }
                    break;
                }
            }
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