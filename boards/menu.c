#include "menu.h"
#include "myADC.h"
#include "timer.h"
#include "board.h"
#include "esp8266.h"
#include "myFlash.h"
#include "fsl_debug_console.h"

#define TIME_OUT    6000    // 2s
#define TIME_IDLE_CHECK 5000 // 200ms
#define TARGET_MODE     0
#define NORMAL_MODE     1

#define CF              1.8
#define F_MODE          1
#define C_MODE          0


void Auto_start_timer_callback(void* ptr);
void Switch_probe_callback(void* ptr);
uint8_t GetNextProbe(void);
void TimeOutCount(void* ptr);
void SelfTest(void* ptr);

#define TARGET_VIEW     0
#define TARGET_PLAY     1

uint8_t target_mode = TARGET_VIEW;

extern temp_t tempBuffer;
static uint8_t probe_number = 0;

extern temp_profile_t temp_profile;

static uint16_t time_out = 0;

uint8_t cmd_count = 0;

wifi_control_t wifi_control = {
    .cmd_id = CMD_WIFI_CONTROL,
    .wifi_mode = WIBBQ_WIFI_STANDALONE
};

wifi_state_t wifi_state = {
    .cmd_id = CMD_WIFI_STATE,
    .wifi_state = 0xB2
};

selftest_t selftest = {
    .cmd_id = CMD_SELT_TEST,
    .power_status = 1,
    .battery_percent = 99,
    .time_stamp = 0xFFF
};

extern monitor_state_t monitor_state;

screen_t* target_screen = NULL;

timer_t auto_switch_probe_timer = NULL;
timer_t time_out_timer = NULL;
timer_t led_in_target_mode_control_timer = NULL;
timer_t auto_start_timer = NULL;

void Menu_Init(void)
{
    RegisterScreen(ROOT_SREEN_ID, RootScreenUpdate, RootScreenKeySetHandle, RootScreenKeyMoveHandle, RootScreenKeyPlusHandle\
        , RootScreenKeyMinusHandle);
    RegisterScreen(TARGET_SCREEN_ID, TargetScreenUpdate, TargetScreenKeySetHandle, TargetScreenKeyMoveHandle, TargetScreenKeyPlusHandle\
        , TargetScreenKeyMinusHandle);
    SetScreenActive(ROOT_SREEN_ID);
    GetCurrentScreen()->notifydatachanged = 1;
    GetCurrentScreen()->mode_data = NORMAL_MODE;

    target_screen = GetScreenContext(TARGET_SCREEN_ID);

    auto_switch_probe_timer = CreateTimer(2500, 1, Switch_probe_callback, NULL);
    auto_start_timer = CreateTimer(10000, 1, Auto_start_timer_callback, NULL);
    time_out_timer = CreateTimer(TIME_IDLE_CHECK, 1, TimeOutCount, NULL);

    SuspendTimer(time_out_timer);
    SuspendTimer(auto_start_timer);
}

void LoadConfigValue(void)
{
    uint8_t i = 0;

    for(i = 0; i < 4; i++)
    {
        temp_profile.temp_probe[i] = (uint16_t)ReadFlash(START_CONFIG_ADDR + (i * 4));
        if (temp_profile.temp_probe[i] >= 350)
        {
            temp_profile.temp_probe[i] = 200;
        }
    }
}

void SaveTargetConfig(void)
{

}

// Root screen
void TimeOutCount(void* ptr)
{
    SetScreenActive(ROOT_SREEN_ID);
    target_mode = TARGET_VIEW;
    SuspendTimer(time_out_timer);
    ResetTimer(time_out_timer);
    ResumeTimer(auto_switch_probe_timer);
}

void Switch_probe_callback(void* ptr)
{
    uint8_t tmp = 0;

    if (probe_number == 0)
    {
        probe_number = GetProbePlugStatus();
    }
    else
    {
        tmp = GetNextProbe();
        if (tmp != probe_number)
        {
            probe_number = tmp;
        }
        else
        {
            probe_number = GetProbePlugStatus();
        }
    }
}

uint8_t GetProbePlugStatus(void)
{
    uint8_t i = 0;

    for(i = 0; i < 4; i++)
    {
        if (GetProbeStatus()[i])
        {
            return i + 1;
        }
    }
    probe_number = 0;
    return 0;
}

uint8_t GetCountOfPlugProbe(void)
{
    uint8_t i = 0;
    uint8_t j = 0;

    for(i =0; i < 4; i++)
    {
        if (GetProbeStatus()[i])
        {
            j++;
        }
    }
    return j;
}

void RootScreenUpdate(void* pdata)
{
    if (GetProbePlugStatus())
    {
        if (!probe_number)
        {
            probe_number = GetProbePlugStatus();
        }
        GetCurrentScreen()->screen_data = GetTemperature(probe_number);
        if (GetCurrentScreen()->deg_mode_data == F_MODE)
        {
            GetCurrentScreen()->screen_data = (uint32_t)((float)(GetCurrentScreen()->screen_data) * CF);
        }
        GetCurrentScreen()->status_data = probe_number;
    }
    else
    {
        GetCurrentScreen()->screen_data = NOP_STR_VIEW;
    }

    GetCurrentScreen()->deg_mode_data = SWITCH_STT();

    GetCurrentScreen()->notifydatachanged = 1;
}

void RootScreenKeySetHandle(uint8_t event)
{
    static uint8_t key_set_hold_flag = 0;
    if (event == KEY_SET_HOLD_EVENT)
    {
        key_set_hold_flag = 1;
        if (wifi_control.wifi_mode >= WIBBQ_WIFI_STANDALONE)
        {
            wifi_control.wifi_mode = WIBBQ_WIFI_CLIENT;
        }
        else
        {
            wifi_control.wifi_mode++;
        }
        PushDataToQueue((void*)&wifi_control, sizeof(wifi_control_t), CMD_NULL);
    }
    else if (event == KEY_SET_RELEASE_EVENT)
    {
        if (!key_set_hold_flag) // ignored this if hold event orcur
        {
        }
        else
        {
            key_set_hold_flag = 0;
        }
    }
}

void RootScreenKeyMoveHandle(uint8_t event)
{
    uint8_t key_move_hold_flag = 0;

    if (event == KEY_MOVE_PRESS_EVENT)
    {
    }
    else if (event == KEY_MOVE_RELEASE_EVENT)
    {
        if (! probe_number)
        {
            return;
        }
        probe_number = GetProbePlugStatus();
        SetScreenActive(TARGET_SCREEN_ID);
        GetCurrentScreen()->screen_data = temp_profile.temp_probe[probe_number - 1];
        GetCurrentScreen()->status_data = probe_number;
        GetCurrentScreen()->mode_data = TARGET_MODE;
        GetCurrentScreen()->notifydatachanged = 1;
        SuspendTimer(auto_switch_probe_timer);
        ResumeTimer(time_out_timer);
    }
    else if(event == KEY_MOVE_HOLD_EVENT) // start or stop monitor
    {
        // set hold flag
        key_move_hold_flag = 1;

        if (monitor_state.state == kStopMonitor)
        {
            monitor_state.state = kStartMonitor;
        }
        else
        {
            monitor_state.state = kStopMonitor;
            StopMonitor();
        }
        PushDataToQueue((void*)&monitor_state, sizeof(monitor_state_t), CMD_NULL);
    }
}

uint8_t GetNextProbe(void)
{
    uint8_t i = 0;
    if (probe_number == 4)
    {
        return 4;
    }

    for (i = probe_number + 1; i < 5; i++)
    {
        if (GetProbeStatus()[i - 1])
        {
            return i;
        }
    }
    return probe_number;
}

uint8_t GetLastProbe(void)
{
    uint8_t i = 0;
    if (probe_number == 1)
    {
        return 1;
    }

    for (i = probe_number - 1; i > 0; i--)
    {
        if (GetProbeStatus()[i - 1])
        {
            return i;
        }
    }
    return probe_number;
}

void RootScreenKeyMinusHandle(uint8_t event)
{
    if (event == KEY_MINUS_PRESS_EVENT)
    {
        ResetTimer(auto_switch_probe_timer);
        probe_number = GetLastProbe();
        if (!probe_number)
        {
            return; // return if no probe pluged
        }
        GetCurrentScreen()->screen_data = GetTemperature(probe_number);
        GetCurrentScreen()->status_data = probe_number;
        GetCurrentScreen()->notifydatachanged = 1;
    }
    else if (event == KEY_MINUS_HOLD_EVENT)
    {
        GetCurrentScreen()->enable_auto_update = 0;// disable auto call update screen fucntion
        CreateTimer(500, 1, SelfTest, NULL);
        SelfTest(NULL);
    }
}

void RootScreenKeyPlusHandle(uint8_t event)
{
    if (event == KEY_PLUS_PRESS_EVENT) // test
    {
        ResetTimer(auto_switch_probe_timer);
        probe_number = GetNextProbe();
        GetCurrentScreen()->screen_data = GetTemperature(probe_number);
        GetCurrentScreen()->status_data = probe_number;
        GetCurrentScreen()->notifydatachanged = 1;
    }
}

// Target screen

void Auto_start_timer_callback(void* ptr)
{
    DBG("Start monitor");
    SuspendTimer(auto_start_timer);
    ResetTimer(auto_start_timer);
    monitor_state.state = kStartMonitor;
    PushDataToQueue((void*)&monitor_state, sizeof(monitor_state_t), monitor_state.cmd_id);
}

void TargetScreenUpdate(void* pdata)
{
    if (!GetProbePlugStatus())
    {
        // back to root screen
        SetScreenActive(ROOT_SREEN_ID);
        GetCurrentScreen()->notifydatachanged = 1;

        target_mode = TARGET_VIEW;
    }

    GetCurrentScreen()->deg_mode_data = SWITCH_STT();
}

void TargetScreenKeySetHandle(uint8_t event)
{
    uint8_t tmp = 0;
    ResetTimer(time_out_timer);
    if (event == KEY_SET_PRESS_EVENT)
    {
        if (target_mode == TARGET_VIEW)
        {
            target_mode = TARGET_PLAY;
            probe_number = GetProbePlugStatus();
        }
        else
        {
            temp_profile.temp_probe[probe_number - 1] = GetCurrentScreen()->screen_data;
            tmp = GetNextProbe();
            if (! tmp)
            {
                //
                target_mode = TARGET_VIEW;
                // back to root
                SuspendTimer(time_out_timer);
                SetScreenActive(ROOT_SREEN_ID);
            }
            else
            {
                if (tmp == probe_number)
                {
                    target_mode = TARGET_VIEW;
                    if (monitor_state.state == kStopMonitor && !(TimerIsRunning(auto_start_timer)))
                    {
                        // auto start after 10s
                        ResumeTimer(auto_start_timer);
                    }
                    DBG("Finish config target, auto start after 10s");
                    ResumeTimer(time_out_timer);
                }
                else
                {
                    probe_number = tmp;
                }
                GetCurrentScreen()->screen_data = temp_profile.temp_probe[probe_number - 1];
                GetCurrentScreen()->status_data = probe_number;
            }
            GetCurrentScreen()->notifydatachanged = 1;
        }
    }
}

void TargetScreenKeyMoveHandle(uint8_t event)
{
    ResetTimer(time_out_timer);
    if (event == KEY_MOVE_RELEASE_EVENT)
    {
        if (target_mode == TARGET_VIEW)
        {
            // back to root
            SuspendTimer(time_out_timer);
            SetScreenActive(ROOT_SREEN_ID);
            SuspendTimer(time_out_timer);
            ResumeTimer(auto_switch_probe_timer);
            GetCurrentScreen()->notifydatachanged = 1;
        }
        else
        {
            target_mode = TARGET_VIEW;
            // save current data to profile
            temp_profile.temp_probe[probe_number - 1] = GetCurrentScreen()->screen_data;
        }
    }
}

void TargetScreenKeyMinusHandle(uint8_t event)
{
    if (event == KEY_MINUS_PRESS_EVENT)
    {
        if (target_mode == TARGET_VIEW)
        {
            probe_number = GetLastProbe();
            GetCurrentScreen()->screen_data = temp_profile.temp_probe[probe_number - 1];
            GetCurrentScreen()->status_data = probe_number;
            GetCurrentScreen()->notifydatachanged = 1;
        }
        else
        {
            if (GetCurrentScreen()->screen_data > STANDAR_TEMP)
            {
                GetCurrentScreen()->screen_data--;
                GetCurrentScreen()->notifydatachanged = 1;
            }
        }
    }
    else if (event == KEY_MINUS_ON_HOLD_EVENT)
    {
        if (target_mode == TARGET_PLAY)
        {
            if (GetCurrentScreen()->screen_data > STANDAR_TEMP)
            {
                GetCurrentScreen()->screen_data -= 5;
                GetCurrentScreen()->notifydatachanged = 1;
            }
        }
    }
    ResetTimer(time_out_timer);
}

void TargetScreenKeyPlusHandle(uint8_t event)
{
    if (event == KEY_PLUS_PRESS_EVENT)
    {
        if (target_mode == TARGET_VIEW)
        {
            probe_number = GetNextProbe();
            GetCurrentScreen()->screen_data = temp_profile.temp_probe[probe_number - 1];
            GetCurrentScreen()->status_data = probe_number;
            GetCurrentScreen()->notifydatachanged = 1;
        }
        else
        {
            if (GetCurrentScreen()->screen_data < MAX_TEMP)
            {
                GetCurrentScreen()->screen_data++;
                GetCurrentScreen()->notifydatachanged = 1;
            }
        }
    }
    else if (event == KEY_PLUS_ON_HOLD_EVENT)
    {
        if (target_mode == TARGET_PLAY)
        {
            if (GetCurrentScreen()->screen_data < MAX_TEMP)
            {
                GetCurrentScreen()->screen_data += 5;
                GetCurrentScreen()->notifydatachanged = 1;
            }
        }
    }
    ResetTimer(time_out_timer);
}

// Self test

void SelfTest(void* ptr)
{
    static uint16_t i = 0;
    uint8_t j = 0;

    GetCurrentScreen()->key_lock = 1; // disable keyboard
    GetCurrentScreen()->enable_auto_update = 0;
    if ((i % 2) == 0)
    {
        GetCurrentScreen()->screen_data = 888;
    }
    else
    {
        GetCurrentScreen()->screen_data = OFF_CODE;
    }
    GetCurrentScreen()->status_data = i % 5;
    GetCurrentScreen()->notifydatachanged = 1;

    if (i++ == 10)
    {
        i = 0;
        for(j = 0; j < 4; j++){
            selftest.probeVal[j] = GetTemperature(j); // fill data
        }
        selftest.power_status = 1;
        selftest.battery_percent = GetBatValue();
        PushDataToQueue((void*)&selftest, sizeof(selftest), CMD_NULL);
        CancelTimer(SelfTest);// finish self test
        GetCurrentScreen()->key_lock = 0; // re-enable keyboard
        GetCurrentScreen()->enable_auto_update = 1; //
        GetCurrentScreen()->status_data = probe_number; //
    }
}
