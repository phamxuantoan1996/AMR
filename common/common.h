#ifndef _COMMON_H
#define _COMMON_H
#include <stdint.h>
#include <cstdint>
#include <semaphore.h>
#include <string>

#define SHM_AMR_STATE_NAME "/shm_amr_status"
constexpr size_t AMR_STATUS_MAX_PAYLOAD = 1024 * 5; //
constexpr size_t SEER_STATE_SIZE = 1024*3;
constexpr uint64_t SHM_AMR_STATE_MAGIC = 0xA5A5A5A5A5A5A5A5ULL;

#define MISSION_BUFFER_SIZE 512

#define MQ_MISSION_REQ "/mission_req_mq"
#define MQ_MISSION_RES "/mission_res_mq"
#define MQ_SIGNAL_CTRL "/signal_ctrl_mq"

enum MISSION_CONFIRM_TYPE {
    MISSION_CONFIRM_NONE,MISSION_CONFIRM_RUN,MISSION_CONFIRM_COMPLETE,MISSION_CONFIRM_CANCEL,
};

enum ROBOT_STATE {
    ROBOT_STATE_NONE,ROBOT_STATE_IDLE,ROBOT_STATE_ACTIVE,ROBOT_STATE_STOP,ROBOT_STATE_ERROR
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

struct Mission_Active_Info_Structure
{
    /* data */
    int32_t mission_code;
    int32_t action_index;
    MISSION_STATE mission_state;
    MISSION_ERROR_CODE mission_error_code;
};

struct AMR_State_Structure
{
    /* data */
    char seer_state[SEER_STATE_SIZE];
    COMMODITY_STATE commodity_state;
    LIFT_LEVEL lift_state;
    ROBOT_STATE robot_state;
    Mission_Active_Info_Structure mission_active_info;
};


#endif
