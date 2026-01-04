#ifndef _COMMON_H
#define _COMMON_H
#include <stdint.h>

#define MISSION_BUFFER_SIZE 512

#define MQ_MISSION_REQ "/mission_req_mq"
#define MQ_MISSION_RES "/mission_res_mq"
#define MQ_SIGNAL_CTRL "/signal_ctrl_mq"

enum MISSION_CONFIRM_TYPE {
    MISSION_CONFIRM_NONE,MISSION_CONFIRM_RUN,MISSION_CONFIRM_COMPLETE,MISSION_CONFIRM_CANCEL,
};

enum MISSION_STATE {
    MISSION_STATE_NONE,MISSION_STATE_RUNNING,MISSION_STATE_COMPLETE,MISSION_STATE_CANCEL,MISSION_STATE_SUSPEND
};

enum LIFT_LEVEL {
    LIFT_NONE,LIFT_LEVEL_0,LIFT_LEVEL_1
};

enum COMMODITY_STATE {
    COMMODITY_NONE,COMMODITY_AVAI,COMMODITY_UNAVAI
};

enum MISSION_ERROR_CODE {
    MISSION_ERROR_NONE,MISSION_ERROR_EMC,MISSION_ERROR_QRCODE,MISSION_ERROR_LIFT,MISSION_ERROR_PATH
};

enum WORK_MODE {
    MANUAL,AUTO
};

struct Mission_Request_Structure
{
    /* data */
    uint32_t mission_code;
    char mission_details[MISSION_BUFFER_SIZE];
};

struct Mission_Response_Structure
{
    /* data */
    uint32_t mission_code;
    MISSION_CONFIRM_TYPE mission_confirm;
};

struct AMR_Relocation_Data_Structure
{
    /* data */
    float x;
    float y;
    float angle;
    float length;
    bool valid;
};

struct Mission_Control_Signal_Structure
{
    /* data */
    bool signal_pause_manual;
    bool signal_resume_manual;
    bool signal_pause_auto;
    bool signal_resume_auto;
    bool signal_cancel;
};


struct AMR_Signal_Control_Structure
{
    /* data */
    AMR_Relocation_Data_Structure signal_relocation;
    LIFT_LEVEL signal_lift;
    Mission_Control_Signal_Structure signal_mission;
};


#endif
