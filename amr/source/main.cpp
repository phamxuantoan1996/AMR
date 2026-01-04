#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include <cerrno>
#include <signal.h>

#include <mqueue.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <jsoncpp/json/json.h>
#include <sstream>

#include <common.h>

std::atomic<bool> running{true};

mqd_t mq_mission_req;
mqd_t mq_mission_res;
mqd_t mq_signal_control;

Mission_Control_Signal_Structure mission_signal_control;

void terminal_sig_callback(int) 
{ 
    running = false; 

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

int main(int argc,char *argv[])
{
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

    std::thread thread_exec_mission(thread_exec_mission_func);
    thread_exec_mission.detach();

    std::thread thread_exec_signal(thread_exec_signal_func);
    thread_exec_signal.detach();

    while (running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    mq_close(mq_mission_req);
    mq_unlink(MQ_MISSION_REQ);
    mq_close(mq_signal_control);
    mq_unlink(MQ_SIGNAL_CTRL);
    return 0;
}