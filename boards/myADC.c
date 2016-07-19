#include "fsl_debug_console.h"
#include "myADC.h"
#include "fsl_rtc.h"
#include "fsl_adc16.h"
#include "board.h"
#include "timer.h"
#include "esp8266.h"
#include "myUI.h"


#define SIZE_REF_ARR    66
#define STANDAR_TEMP    25
#define MAX_TEMP        350
#define TEMP_STEP       5
#define ZERO_ADC_VALUE  100

#define LOW_TEMP_NORMAL_CASE    500 //
#define TEMP_RISE_STEP          50  // deg/s
#define TIME_TEMP_REACH         10000 // 10s

#define ADC_VAL_LOW_BAT         35746
#define ADC_VAL_FULL_BAT        41704
#define ADC_VAL_EMPTY_BAT       35746

#define NO_ALARM            0
#define REACH_ALARM         (uint8_t)(1 << 1)
#define DURATION_ALARM      (uint8_t)(1 << 2)
#define LOW_T_ALARM         (uint8_t)(1 << 3)
#define HIGH_T_ALARM        (uint8_t)(1 << 4)
#define SKIP_ALARM          255

static adc16_channel_config_t adcChnConfig = {0};
static adcHandle_t adchandler = {0};
static uint16_t g_temperature[4] = {0};
temp_t tempBuffer = {0};
static uint16_t g_BufferAvg[4] = {0};
static uint16_t g_BufferAvgIndex = 0;
static uint8_t  g_BufferIndex = 0;

static uint16_t ADC_Ref[SIZE_REF_ARR] = {617,791,1005,1266,1582,1961,2411,2942,3563,4282,5108,6047,7106,8289,\
    9598,11031,12585,14254,16029,17896,19842,21850,23902,25979,28063,30137,32183,34186,36133,38014,\
    39819,41542,43178,44725,46182,47548,48827,50020,51130,52161,53117,54002,54822,55579,56278,56923,\
    57519,58069,58576,59044,59476,59876,60244,60585,60900,61191,61460,61710,61941,62156,62355,62540,\
    62712,62872,63021,63159};

timer_t adc_timer = NULL;
timer_t alarm_timer = NULL;

adc_calib_t adc_calib = {
    .cmd_id = CMD_GET_CALIB,
    .calib_result[0] = 0,
    .calib_result[1] = 0,
    .calib_result[2] = 0,
    .calib_result[3] = 0
};

battery_t bat_info = {
    .cmd_id = CMD_BATTERY,
    .bat_status = 0,
    .bat_capacity = 0
};

uint8_t Probe_plug_status[4] = {0};

static uint32_t low_temp_time_count[4] = {0};
static uint32_t burning_time_count[4] = {0};
static uint32_t probe_duration[4] = {0};
static uint32_t probe_duration_start[4] = {0};
static uint16_t last_temperature[4] = {0};
static uint16_t start_temperature[4] = {0};
static uint8_t alarm_flag[4] = {0};
static uint8_t is_alarm = 0;
static uint8_t alarm_id = 0;

static uint8_t start_flag[4] = {0};

static temp_done_t temp_done = {
    .cmd_id =  CMD_TEMP_DONE
};

static time_done_t time_done = {
    .cmd_id =  CMD_TIME_DONE
};

alarm_done_t alarm_mess = {
    .cmd_id = CMD_ALARM_DONE
};

burning_t burning = {
    .cmd_id = CMD_BURNNING
};

battery_low_t bat_low = {
    .cmd_id = CMD_BATTERY_LOW
};

low_temp_t low_temp = {
    .cmd_id = CMD_LOW_TEMP
};

extern monitor_state_t monitor_state;
extern temp_profile_t temp_profile;
extern rtc_datetime_t datetime;

void AlarmTest(void* ptr);
void Alarm_callback(void* ptr);

void ADCInit(void)
{
    //uint16_t offsetValue = 0; /*!< Offset error from correction value. */
    adc16_config_t adcUserConfig;

    //offsetValue = BOARD_ADC_BASEADDR->OFS;
    //ADC16_SetOffsetValue(BOARD_ADC_BASEADDR, offsetValue);

    ADC16_GetDefaultConfig(&adcUserConfig);
    adcUserConfig.resolution = kADC16_Resolution16Bit;
    adcUserConfig.enableContinuousConversion = false;
    adcUserConfig.clockSource = kADC16_ClockSourceAsynchronousClock;
    adcUserConfig.longSampleMode = kADC16_LongSampleCycle24;
    adcUserConfig.enableLowPower = 1;

    ADC16_Init(BOARD_ADC_BASEADDR, &adcUserConfig);
    /* Auto calibration */
    ADC16_DoAutoCalibration(BOARD_ADC_BASEADDR);
    /* disable hardware trigger  */
    ADC16_EnableHardwareTrigger(BOARD_ADC_BASEADDR, false);

    adcChnConfig.channelNumber = ADC_CHANNEL_PROBE1;
    adcChnConfig.enableInterruptOnConversionCompleted = true;

    //memset(adchandler, 0, sizeof(adchandler));

    adchandler.channelArray[0] = ADC_CHANNEL_PROBE1;
    adchandler.channelArray[1] = ADC_CHANNEL_PROBE2;
    adchandler.channelArray[2] = ADC_CHANNEL_PROBE3;
    adchandler.channelArray[3] = ADC_CHANNEL_PROBE4;
    adchandler.channelArray[4] = ADC_CHANNEL_BAT;
    adchandler.convertDone = 0;
    adchandler.currentChannel = 0;

   /* Enable IRQ. */
    NVIC_EnableIRQ(ADC0_IRQn);

    tempBuffer.cmd_id = CMD_TEMP;

    SetEventHookCallback(ClearAlarm);

    // create timer for alarm
    alarm_timer = CreateTimer(200, 1, Alarm_callback, NULL);
    SuspendTimer(alarm_timer);
}

void Alarm_callback(void* ptr)
{
    static uint8_t isOn = 0;
    // Blink red led and green led(normal and current led)
    // On/Off buzzer
    if (isOn)
    {
        BUZZER_ON();
        // set led current and target on
        isOn = 0;
    }
    else
    {
        BUZZER_OFF();
        isOn = 1;
    }
}

uint16_t GetTemperature(uint8_t probe)
{
   return g_temperature[probe - 1];
}

uint8_t GetBatValue(void)
{
    uint8_t retVal = 0;
    if (adchandler.result.vbat < ADC_VAL_EMPTY_BAT)
    {
        return 0;
    }
    retVal =  (adchandler.result.vbat - ADC_VAL_EMPTY_BAT) * 100 / (ADC_VAL_FULL_BAT - ADC_VAL_EMPTY_BAT);

    return retVal;
}

uint8_t* GetTemperaturesBuffer(void)
{
    g_BufferIndex = 0;
    return (uint8_t*) &tempBuffer;
}

uint8_t GetTemperaturesBufferLen(void)
{
    return sizeof(tempBuffer);
}

uint16_t Calc_Temperatures(uint16_t _adcVal, uint16_t _adc_offset)
{
    uint8_t i = 0;
    float tmp = 0;

    uint16_t adc_val_tmp = _adcVal - _adc_offset;

    // find adc range
    for(i = 0; i < SIZE_REF_ARR; i++)
    {
        if (ADC_Ref[i] >= adc_val_tmp)
        {
            break;
        }
    }

    if (i == 0) // too low temp
    {
        return (STANDAR_TEMP);
    }
    else if (i == SIZE_REF_ARR) // to high temp
    {
        return (MAX_TEMP);
    }
    else // in range
    {
        tmp = ((float)adc_val_tmp - (float)ADC_Ref[i-1]) * TEMP_STEP / ((float)ADC_Ref[i] - (float)ADC_Ref[i-1]);
        tmp += (i - 1) * TEMP_STEP + STANDAR_TEMP;
        return (uint16_t)tmp;
    }
}

uint8_t ClearAlarm(uint8_t event)
{
    uint8_t i;

    if (is_alarm)
    {
        DBG("Clear alarm");
        // off buzzer
        SuspendTimer(alarm_timer);
        BUZZER_OFF();

        is_alarm = 0;
        alarm_mess.alarm_id = alarm_id;
        alarm_id = 0;

        ScreenSetAutoOff(1);

        //if (event != NO_KEY_EVENT)
        {
            PushDataToQueue((void*)&alarm_mess, sizeof(alarm_done_t), CMD_NULL);
        }

        return 1;
    }
    return 0;
}

uint8_t GetAlarmStatus(void)
{
    return is_alarm;
}

void AlarmTest(void* ptr)
{
    static uint8_t i = 0;

    if (i)
    {
        LED_BAT(LED_FULL);
        i = 0;
    }
    else
    {
        LED_BAT(LED_OFF);
        i = 1;
    }
}

uint8_t* GetProbeStatus(void)
{
    return Probe_plug_status;
}

void BatProcess(void)
{
    static uint32_t current_time = 0;

    bat_info.bat_capacity = GetBatValue();
    // send bat info per 60s
    if (GetTimerTick() - current_time >= 60000)
    {
        current_time = GetTimerTick();

        // send bat stt
        bat_info.bat_status = 0;
        PushDataToQueue((void*)&bat_info, sizeof(battery_t), CMD_NULL);
        // check bat
        // < 10 %, send low bat alarm
        if (bat_info.bat_capacity < 10)
        {
            bat_low.battery = bat_info.bat_capacity;
            PushDataToQueue((void*)&bat_low, sizeof(battery_low_t), CMD_NULL);
        }
    }
}

void MonitorProcess(void)
{
    uint8_t i = 0;
    for(i = 0; i < 4; i++)
    {
        if (Probe_plug_status[i] == 0) // check which the probe is plugged
        {
            continue;
        }
        // enable alarm flag here
        if (start_flag[i] == 0) // init before start
        {
            start_flag[i] = 1;
            alarm_flag[i] = NO_ALARM;
            last_temperature[i] = g_temperature[i];
            start_temperature[i] = g_temperature[i];
            low_temp_time_count[i] = GetTimerTick();
        }

        if (probe_duration_start[i] == 0)
        {
            probe_duration_start[i] = GetTimerTick(); // get start time
        }
        else
        {
            probe_duration[i] = GetTimerTick() - probe_duration_start[i]; // calc duration time

            if (probe_duration[i] >= (temp_profile.duration[i] * 1000))
            {
                // duration reach
                // buzzer
                if ((alarm_flag[i] & DURATION_ALARM)!= DURATION_ALARM)
                {
                    DBG("Duration alarm");
                    //
                    is_alarm = 1;

                    alarm_flag[i] |= DURATION_ALARM;
                    alarm_id = CMD_TIME_DONE;
                    time_done.probe = i;
                    time_done.duration = probe_duration[i];
                    time_done.temp = g_temperature[i];

                    RTC_GetDatetime(RTC, &datetime);

                    time_done.clock = RTC_ConvertDatetimeToSeconds(&datetime);

                    PushDataToQueue((void*)&time_done, sizeof(time_done_t), CMD_NULL);
                }
            }
        }

        // very fast rising temperature
        if(g_temperature[i] - last_temperature[i] > TEMP_RISE_STEP) // 200 - 250 ms
        {
            DBG("Very fast rising temperature");
            if ((alarm_flag[i] & HIGH_T_ALARM) != HIGH_T_ALARM)
            {
                alarm_flag[i] |= HIGH_T_ALARM;
                // Set buzzer
                // implement me
                // put data to queue
                is_alarm = 0;
                alarm_id = CMD_BURNNING;
                burning.probe = i;
                burning.duration = probe_duration[i];
                burning.temp = g_temperature[i];
                PushDataToQueue((void*)&burning, sizeof(burning_t), CMD_NULL);
            }
        }

        last_temperature[i] = g_temperature[i];

        // low temp
        if (g_temperature[i] - start_temperature[i] >= 5)
        {
            low_temp_time_count[i] = GetTimerTick();
        }

        if (GetTimerTick() - low_temp_time_count[i] >= 1800000) // 30 minutes
        {
            if ((alarm_flag[i] & LOW_T_ALARM) != LOW_T_ALARM)
            {
                DBG("Low temp alarm");
                is_alarm = 1;
                alarm_flag[i] |= LOW_T_ALARM;
                // Set buzzer
                // implement me
                // put data to queue
                alarm_id = CMD_LOW_TEMP;
                low_temp.probe = i;
                low_temp.duration = probe_duration[i];
                low_temp.start_temp = start_temperature[i];
                low_temp.current_temp = g_temperature[i];

                RTC_GetDatetime(RTC, &datetime);
                low_temp.clock = RTC_ConvertDatetimeToSeconds(&datetime);

                PushDataToQueue((void*)&low_temp, sizeof(low_temp_t), CMD_NULL);
            }
        }

        // reach temp
        if (g_temperature[i] >= temp_profile.temp_probe[i]) // reach temp case
        {
            if (GetTimerTick() - burning_time_count[i] >= TIME_TEMP_REACH)
            {
                // reach temp
                // alarm buzzer
                // send mess to ESP

                if ((alarm_flag[i] & REACH_ALARM) != REACH_ALARM)
                {
                    DBG("Reach temperature alarm");
                    is_alarm = 1;
                    alarm_flag[i] |= REACH_ALARM;
                    // Set buzzer
                    // implement me
                    // put data to queue
                    alarm_id = CMD_TEMP_DONE;
                    temp_done.probe = i;
                    temp_done.duration = probe_duration[i];
                    temp_done.temp = g_temperature[i];

                    RTC_GetDatetime(RTC, &datetime);
                    temp_done.clock = RTC_ConvertDatetimeToSeconds(&datetime);

                    PushDataToQueue((void*)&temp_done, sizeof(temp_done_t), CMD_NULL);
                }
            }
        }
        else
        {
            burning_time_count[i] = GetTimerTick();
        }
    }

    if (is_alarm && !TimerIsRunning(alarm_timer))
    {
        SetIdleMode(kRun);
        ScreenSetAutoOff(0);

        // start timer alarm
        ResumeTimer(alarm_timer);
        DBG("Start timer alarm");
    }
}

void StopMonitor(void)
{
    uint8_t i = 0;
    for(i = 0; i < 4; i++)
    {
        probe_duration[i] = probe_duration_start[i] = 0; // clear all time data
        start_flag[i] = 0;
        if (Probe_plug_status[i])
        {
            alarm_flag[i] = NO_ALARM; // reset alarm flag
        }
    }
}


void ADCProcess(void)
{
    uint8_t i = 0;

    if (adchandler.convertDone == 1)
    {
        adchandler.convertDone = 0;

        for(i = 0; i < 4; i++)
        {
            if (adchandler.resultArr[i] <= ZERO_ADC_VALUE) // check the probe is pluged or unpluged
            {
                Probe_plug_status[i] = 0;
            }
            else
            {
                if (Probe_plug_status[i] == 0)
                {
                    SetIdleMode(kRun);
                    Probe_plug_status[i] = 1;
                }
            }
            g_temperature[i] = Calc_Temperatures(adchandler.resultArr[i], adc_calib.calib_result[i]);
            g_BufferAvg[i] += g_temperature[i];
        }

        // Bat
        BatProcess();

        if (monitor_state.state == kStartMonitor) // start monitor
        {
            MonitorProcess();
        }
        else
        {
            StopMonitor(); // reset alarm info
        }


        if (g_BufferAvgIndex++ >= 5) // calc agv temp value in 1s, sample 5 times per second
        {
            g_BufferAvgIndex = 0;
            for (i = 0; i < 4; ++i)
            {
                g_BufferAvg[i] = g_BufferAvg[i] / 5;
            }
            for(i = 0; i < 4; i++)
            {
                tempBuffer.temp_probe[i + g_BufferIndex] = g_BufferAvg[i];
                g_BufferAvg[i] = 0;
            }
            if (g_BufferIndex >= 120)
            {
                if (monitor_state.state)
                {
                    RTC_GetDatetime(RTC, &datetime);
                    tempBuffer.time_stamp = RTC_ConvertDatetimeToSeconds(&datetime);
                    PushDataToQueue((void*)&tempBuffer, sizeof(tempBuffer), CMD_NULL);
                }
                g_BufferIndex = 0;
            }
            g_BufferIndex += 4;
        }
    }
}

void ADC_Calib(void)
{
    uint8_t count = 0;
    uint8_t i = 0;
    uint32_t sumAdcVal = 0;

    // wait for previous trigger
    while(adchandler.currentChannel != 0);

    adchandler.convertDone = 0;

    while(count < 5) // sample 5 times
    {
        ADCTrigger(NULL);

        while(adchandler.convertDone == 0); // wait convert done

        for (i = 0; i < 4; i++)
        {
            adc_calib.calib_result[i] += adchandler.resultArr[i];
        }
    }

    for (i = 0; i < 4; i++)
    {
        adc_calib.calib_result[i] /= 5;
    }

    for (i = 0; i < 4; i++) sumAdcVal += adc_calib.calib_result[i];

    sumAdcVal /= 4;

    for (i = 0; i < 4; i++) {
        adc_calib.calib_result[i] = sumAdcVal - adc_calib.calib_result[i];
    }

    PushDataToQueue((void*)&adc_calib, sizeof(adc_calib_t), CMD_NULL);
}

void ADCTrigger(void* pdata)
{
    /* Configure channel 0 */
    ADC16_SetChannelConfig(BOARD_ADC_BASEADDR, BOARD_ADC_CHANNEL_GROUP, &adcChnConfig);
}

void ADC_StartProcess(void)
{
    adc_timer = CreateTimer(200, 1 , ADCTrigger, NULL);// Create adc handle every 200ms
}

void ADC0_IRQHandler(void)
{
    /* Get current ADC value */
    adchandler.resultArr[adchandler.currentChannel] = ADC16_GetChannelConversionValue(BOARD_ADC_BASEADDR, BOARD_ADC_CHANNEL_GROUP);

    if (adchandler.currentChannel == 4)
    {
        adchandler.convertDone = 1;
        adchandler.currentChannel = 0;
        adcChnConfig.channelNumber = adchandler.channelArray[adchandler.currentChannel];
    }
    else
    {
        adchandler.currentChannel++;
        adcChnConfig.channelNumber = adchandler.channelArray[adchandler.currentChannel];
        ADC16_SetChannelConfig(BOARD_ADC_BASEADDR, BOARD_ADC_CHANNEL_GROUP, &adcChnConfig);
    }
}
