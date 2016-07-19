#include "myLed.h"
#include "board.h"
#include "timer.h"

//static void LedPutData(uint8_t data);
static void LedOff(void* ptr);
static void LedOn(void* ptr);
void LedStatusPutData(uint8_t pos);

static uint16_t g_data = 0;
static uint8_t g_led1_data = 0;
static uint8_t g_led2_data = 0;
static uint8_t g_led3_data = 0;
static uint8_t g_current_pos = 1;
static uint8_t g_enable = 1;
static uint8_t numArray[10] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
static uint8_t str_no_p[3] = {0x37, 0x5c, 0x73};
static uint8_t str_err[3] = {0x79, 0x50, 0x50};
static uint16_t g_onTime = 0;
static uint16_t g_offTime = 0;
static uint8_t g_status_data = 0;
static uint8_t g_deg_f = 0;
static uint8_t g_bat = 0;
static uint8_t g_mode = 0;
static uint8_t enable_flag = 1;

//static led_context_t leds_status[MAX_STATUS] = {0};

extern uint8_t Probe_plug_status[4];

static void LedOn(void* ptr)
{
    LedEnable(true);
    CreateTimer(g_offTime, 0 , LedOff, NULL);
}

static void LedOff(void* ptr)
{
    LedEnable(false);
    CreateTimer(g_onTime, 0 , LedOn, NULL);
}

uint8_t* GetStr(uint8_t str_id)
{
    if (str_id == STR_ERR)
    {
        return str_err;
    }
    if (str_id == STR_NOP)
    {
        return str_no_p;
    }
    return 0;
}

void LedUpdate(uint32_t data)
{
    if (data == OFF_CODE && enable_flag)
    {
        LedEnable(0);
        enable_flag = 0;
    }
    else if (data == NOP_STR_VIEW)
    {
        g_led1_data = str_no_p[2];
        g_led2_data = str_no_p[1];
        g_led3_data = str_no_p[0];
        g_data = 150;
        return;
    }
    else if (data == ERR_STR_VIEW)
    {
        g_led1_data = str_err[2];
        g_led2_data = str_err[1];
        g_led3_data = str_err[0];
        g_data = 150;
        return;
    }
    else
    {
        if(!enable_flag){
            LedEnable(1);
            enable_flag = 1;
        }
        g_data = data;
    }
    g_led1_data = numArray[g_data % 10];
    g_led2_data = numArray[(g_data % 100) / 10];
    g_led3_data = numArray[g_data / 100];

}

void LedStatusUpdate(uint8_t data)
{
    g_status_data = data;
}

void LedEnableBlink(uint16_t onTime, uint16_t offTime)
{
    g_onTime = onTime;
    g_offTime = offTime;
    CreateTimer(g_onTime, 0 , LedOn, NULL);
}

void LedDisableBlink(void)
{
    g_onTime = 0;
    g_offTime = 0;
    CancelTimer(LedOn);
    CancelTimer(LedOff);
}

void LedDisplayString(string_type_t type, uint8_t time)
{

}

void LedEnable(bool enable)
{
    if (enable)
    {
        g_enable = 1;
    }
    else
    {
        g_enable = 0;
        // off 7 segment led
        CTRL_PUT_DATA(0);
        // off probe status led
        GPIOE->PDOR = (GPIOE->PDOR & ~(0x710000));
        // off bat led
        LED_BAT(LED_OFF);
        // off c d led
        LED_DEG_FAHR(LED_OFF);
        //
        LED_CURRENT_TARGET(LED_OFF);
    }
}

void LedStatusPutData(uint8_t pos)
{
    if (pos == 0)
    {
        GPIOE->PDOR = (GPIOE->PDOR & ~(0x710000));
        return;
    }
    if (pos == 1)
    {
        pos = 16;
    }
    else
    {
        pos += 18;
    }

    GPIOE->PDOR = (GPIOE->PDOR & ~(0x710000)) | (1 << pos);
}

void LedStatusProcess(uint8_t* data, uint8_t pos)
{
    // data: indicate which the probe is pluged
    // pos: blink position
    // GPIO 16 20 21 22

    uint32_t regData = 0;
    static uint16_t period = 0;
    static uint8_t stt_data[4] = {0};
    uint8_t i = 0;

    for(i = 0; i < 4; i++)
    {
        if (pos == (i + 1) && data[i])
        {
            if (period >= 50)
            {
                period = 0;
                stt_data[i] = stt_data[i] ^ data[i];
            }
        }
        else
        {
            stt_data[i] = data[i];
        }
    }

    period++;

    regData = ((uint32_t)stt_data[3] << 22) | ((uint32_t)stt_data[2] << 21) | ((uint32_t)stt_data[1] << 20) | ((uint32_t)stt_data[0] << 16);

    if ((GPIOE->PDOR & 0x710000) != regData)
    {
        GPIOE->PDOR = (GPIOE->PDOR & ~(0x710000)) | (regData);
    }
}

void LedDegSetData(uint8_t data)
{
    g_deg_f = data;
}

void LedBatSetData(uint8_t data)
{
    g_bat = data;
}

void LedModeSetData(uint8_t data)
{
    g_mode = data;
}

void LedProcess(void* pData)
{
    static uint16_t sleep_count = 0;

    if (!g_enable)
    {
        if (sleep_count < 50) // 250 ms
        {
            LED_CURRENT_TARGET(LED_TARGET);
        }
        else
        {
            LED_CURRENT_TARGET(LED_OFF);
        }
        if (sleep_count < 2000)
        {
            sleep_count++;
        }
        else

        {
            sleep_count = 0;
        }
        return;
    }

    sleep_count = 0;

    // push data to led status
    // LedStatusPutData(g_status_data);
    LedStatusProcess(Probe_plug_status, g_status_data);

    // batery status
     if (g_bat)
     {
         LED_BAT(LED_FULL);
     }
     else
     {
         LED_BAT(LED_LOW);
     }

    // C F switch
    if (g_deg_f)
    {
        LED_DEG_FAHR(LED_DEG);
    }
    else
    {
        LED_DEG_FAHR(LED_FAHR);
    }

    // mode target current
    if (g_mode > 1)
    {
        LED_CURRENT_TARGET(LED_CURRENT_TARGET_ON);
    }
    else if (g_mode) LED_CURRENT_TARGET(LED_CURRENT); else LED_CURRENT_TARGET(LED_TARGET);

    switch(g_current_pos)
    {
        // use directly register
        case 1:
          LED_PUT_DATA(g_led1_data); CTRL_PUT_DATA(1); break;
        case 2:
          LED_PUT_DATA(g_led2_data); CTRL_PUT_DATA(2); break;
        case 3:
          LED_PUT_DATA(g_led3_data); CTRL_PUT_DATA(4); break;
    }

    g_current_pos++;

    if (g_data < 100)
    {
        if (g_current_pos > 2) // Just only on two led
        {
            g_current_pos = 1;
        }
    }
    else
    {
        if (g_current_pos > 3) // On three led
        {
            g_current_pos = 1;
        }
    }
}
