#ifndef __MYLED_H
#define __MYLED_H

#include "stdint.h"
#include "stdbool.h"

#define MAX_STATUS 8
#define OFF_CODE 0xFFFF

#define NOP_STR_VIEW     (OFF_CODE - 1)
#define ERR_STR_VIEW     (OFF_CODE - 2)
#define STR_ERR     0
#define STR_NOP     1

typedef enum _string_type
{
    kClient = 0,
    kError1,
    kError2,
    kError3,
    kError4
} string_type_t;

typedef struct _led_context
{
    uint16_t onTime;
    uint16_t offTime;
    uint16_t total_time;
    uint8_t times;
    void (*ledon) (void);
    void (*ledoff) (void);
} led_context_t;

void LedProcess(void* pData);
void LedStatusUpdate(uint8_t data);
void LedDisplayString(string_type_t type, uint8_t time);
void LedEnableBlink(uint16_t onTime, uint16_t offTime);
void LedDisableBlink(void);
void LedUpdate(uint32_t data);
void LedEnable(bool enable);
void LedDegSetData(uint8_t data);
void LedBatSetData(uint8_t data);

uint8_t* GetStr(uint8_t str_id);

void LedDegSetData(uint8_t data);
void LedBatSetData(uint8_t data);
void LedModeSetData(uint8_t data);

#endif
