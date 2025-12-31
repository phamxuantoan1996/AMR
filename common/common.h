#ifndef _COMMON_H
#define _COMMON_H
#include <stdint.h>

#define MISSION_BUFFER_SIZE 512

struct Mission_Request_Structure
{
    /* data */
    uint32_t mission_code;
    uint8_t mission_details[MISSION_BUFFER_SIZE];
};

struct Mission_Response_Structure
{
    /* data */
    uint32_t mission_code;
    bool mission_confirm;
};



#endif
