#ifndef __MYADC_H
#define __MYADC_H


#define SIZE_REF_ARR    66
#define STANDAR_TEMP    25
#define MAX_TEMP        350
#define TEMP_STEP       5

typedef struct
{
    uint8_t cmd_id;
    uint8_t reserve;
    uint32_t time_stamp;
    uint16_t temp_probe[120];
} temp_t;

typedef struct adcHandle
{
    uint8_t convertDone;
    uint8_t channelArray[5];
    uint8_t currentChannel;
    uint8_t reserve;
    uint16_t reserve1;
    union
    {
        uint16_t resultArr[5];
        struct r
        {
            uint16_t probe1;
            uint16_t probe2;
            uint16_t probe3;
            uint16_t probe4;
            uint16_t vbat;
        } result;
    };
}adcHandle_t;

void ADCInit(void);
void ADCTrigger(void* pdata);
void ADCProcess(void);
void ADC_StartProcess(void);
uint8_t GetBatValue(void);
uint16_t GetTemperature(uint8_t probe);
uint8_t* GetTemperaturesBuffer(void);
uint8_t GetTemperaturesBufferLen(void);
uint8_t ClearAlarm(uint8_t event);
uint8_t* GetProbeStatus(void);
void StopMonitor(void);

#endif
