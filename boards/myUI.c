#include "myUI.h"
#include "board.h"
#include "myADC.h"
#include "timer.h"
#include "myLed.h"
#include "fsl_port.h"

#define MAX_KEY         4U

#define TIME_IDLE_CHECK 200U    // 200ms
#define TIME_IDLE       10000U  // 10s
#define COUNT_RATIO     560U    // 560ms
#define TIME_HOLD_COUNT 5000U   // 5s

#define KEY_SET_MASK 0x04
#define KEY_MOVE_MASK 0x01
#define KEY_PLUS_MASK 0x02
#define KEY_MINUS_MASK 0x08

#define KEY_SET     0
#define KEY_MOVE     1
#define KEY_MINUS     2
#define KEY_PLUS     3

static i2c_master_handle_t g_MasterHandle;
static cap1106_handle_t cap1106Handle = {0};
static i2c_master_config_t i2cConfig = {0};
static mode_t mode = kRun;
static uint8_t CAP_state = 0;
volatile uint8_t  key_interrupt = 0;

static uint8_t arr_key_press_event[MAX_KEY] = {0};
static uint8_t arr_key_release_event[MAX_KEY] = {0};
static uint8_t arr_key_hold_event[MAX_KEY] = {0};
static uint8_t arr_key_on_hold_event[MAX_KEY] = {0};
static uint8_t arr_key_data[MAX_KEY] = {0};

static uint32_t start_hold_time = 0;

static uint8_t current_key_event = NO_KEY_EVENT;

// screen
static uint8_t screen_active_id = 0;
static screen_t* screen_active = NULL;
static screen_t screenHolder[MAX_SCREEN] = {0};

// event hook callback
event_hook_handle event_hook = NULL;

// idle process
uint32_t start_idle_time_count = 0;
uint32_t idle_time_ref = TIME_IDLE;
uint8_t enable_auto_off_screen = 1;

// function
timer_t touch_buzzer_timer = NULL;

void Touch_buzzer_timer_callback(void* ptr)
{
    BUZZER_OFF();
    SuspendTimer(touch_buzzer_timer);
    ResetTimer(touch_buzzer_timer);
}

void BuzzerBeep(void)
{
    BUZZER_ON();
    ResumeTimer(touch_buzzer_timer);
}

void UI_Init(void)
{
    cap1106Handle.base = BOARD_I2C_BASEADDR;
    cap1106Handle.i2cHandle = &g_MasterHandle;
    cap1106Handle.xfer.slaveAddress  = 0x28;

    I2C_MasterGetDefaultConfig(&i2cConfig);
    i2cConfig.baudRate_Bps = 200000U;
    I2C_MasterInit(BOARD_I2C_BASEADDR, &i2cConfig, CLOCK_GetFreq(I2C1_CLK_SRC));
    I2C_MasterTransferCreateHandle(BOARD_I2C_BASEADDR, &g_MasterHandle, NULL, NULL);

    CAP1106_Init(&cap1106Handle);

    touch_buzzer_timer = CreateTimer(80, 1, Touch_buzzer_timer_callback, NULL);
    SuspendTimer(touch_buzzer_timer);
}

void SetScreenActive(uint8_t _screen_id)
{
    uint8_t i= 0;

    screen_active_id = _screen_id;
    for(i = 0; i < MAX_SCREEN; i++)
    {
        if (screenHolder[i].screen_id == screen_active_id)
        {
            screen_active = &screenHolder[i];
            break;
        }
    }
}

void RegisterScreen(uint8_t _screen_id, update_handle _update_screen, key_handle _key_set_handle,\
     key_handle _key_move_handle, key_handle _key_plus_handle, key_handle _key_minus_handle)
{
    uint8_t i = 0;
    for(i = 0; i < MAX_SCREEN; i++)
    {
        if (screenHolder[i].screen_id == 0)
        {
            screenHolder[i].screen_id = _screen_id;
            screenHolder[i].screen_data = 0;
            screenHolder[i].status_data = 0;
            screenHolder[i].key_lock = 0;
            screenHolder[i].enable_auto_update = 1;
            screenHolder[i].notifydatachanged = 0;
            screenHolder[i].update_screen =    _update_screen;
            screenHolder[i].key_set_handle =    _key_set_handle;
            screenHolder[i].key_move_handle =   _key_move_handle;
            screenHolder[i].key_plus_handle =   _key_plus_handle;
            screenHolder[i].key_minus_handle =  _key_minus_handle;
            break;
        }
    }
}

screen_t* GetCurrentScreen(void)
{
    return screen_active;
}

screen_t * GetScreenContext(uint8_t screen_id)
{
    uint8_t i = 0;
    for(i = 0; i < MAX_SCREEN; i++)
    {
        if (screenHolder[i].screen_id == screen_id)
        {
            return &screenHolder[i];
        }
    }
    return 0;
}

void ScreenSetAutoOff(uint8_t enable)
{
    enable_auto_off_screen = enable;
    start_idle_time_count = GetTimerTick();
}

void ScreenProcess(void)
{
    if (!screen_active->key_lock) // key is not locking
    {
        switch(current_key_event)
        {
            case KEY_SET_PRESS_EVENT:
            case KEY_SET_RELEASE_EVENT:
            case KEY_SET_HOLD_EVENT:
            case KEY_SET_ON_HOLD_EVENT:
                if (screen_active->key_set_handle)
                {
                    screen_active->key_set_handle(current_key_event);
                }
                break;
            case KEY_MOVE_PRESS_EVENT:
            case KEY_MOVE_RELEASE_EVENT:
            case KEY_MOVE_HOLD_EVENT:
            case KEY_MOVE_ON_HOLD_EVENT:
                if (screen_active->key_move_handle)
                {
                    screen_active->key_move_handle(current_key_event);
                }
                break;
            case KEY_PLUS_PRESS_EVENT:
            case KEY_PLUS_RELEASE_EVENT:
            case KEY_PLUS_HOLD_EVENT:
            case KEY_PLUS_ON_HOLD_EVENT:
                if (screen_active->key_plus_handle)
                {
                    screen_active->key_plus_handle(current_key_event);
                }
                break;
            case KEY_MINUS_PRESS_EVENT:
            case KEY_MINUS_RELEASE_EVENT:
            case KEY_MINUS_HOLD_EVENT:
            case KEY_MINUS_ON_HOLD_EVENT:
                if (screen_active->key_minus_handle)
                {
                    screen_active->key_minus_handle(current_key_event);
                }
                break;
        }
    }
    if (screen_active->notifydatachanged)
    {
        screen_active->notifydatachanged = 0;
        if(screen_active->update_screen && screen_active->enable_auto_update)
        {
            screen_active->update_screen(NULL);
        }
        LedUpdate(screen_active->screen_data);
        LedStatusUpdate(screen_active->status_data);
        LedDegSetData(screen_active->deg_mode_data);
        LedBatSetData(screen_active->bat_status);
        LedModeSetData(screen_active->mode_data);
    }
}

void SetEventHookCallback(event_hook_handle handle)
{
    event_hook = handle;
}

void EmitKeyEvent(void)
{
    uint8_t i = 0;

    if (event_hook)
    {
        if (event_hook(current_key_event)) // return false
        {
            return;
        }
    }
    for(i = 0; i < MAX_KEY; i++)
    {
        if (arr_key_press_event[i])
        {
            current_key_event = i * 3;
            BuzzerBeep();
            break;
        }
        else if (arr_key_hold_event[i] == 1)
        {
            arr_key_hold_event[i] = 2;
            current_key_event = i * 3 + 2;
            BuzzerBeep();
            break;
        }
        else if (arr_key_release_event[i])
        {
            current_key_event = i * 3 + 1;
            break;
        }
        else if (arr_key_on_hold_event[i])
        {
            current_key_event = i * 3 + 2 + 20;
        }
    }
}

void BuzzerOff(void* prt)
{
    // replacement the led by buzzer
    LED_DEG_FAHR(LED_OFF);
}

void SetIdleMode(mode_t _mode)
{
    LedEnable(_mode == kRun); // on led
    mode = _mode;
    start_idle_time_count = GetTimerTick();
}

void UIProcess(void)
{
    uint8_t i = 0;
    uint8_t latchedState = 0;
    uint8_t current_state = 0;
    uint8_t key_data_press = 0;
    uint8_t key_data_release = 0;

    if(key_interrupt)
    {
        key_interrupt = 0;

        CAP1106_ReadReg(&cap1106Handle, CAP1106_REG_SENSOR_INPUT, 1, &latchedState);
        CAP1106_ResetStatus();
        CAP1106_ReadReg(&cap1106Handle, CAP1106_REG_SENSOR_INPUT, 1, &current_state);

        // If a press occurred that was not already registered
        // by the CAP_state variable, the XOR of the latched
        // state and the CAP_state will output the sensors
        // that were pressed.
        key_data_press |= latchedState ^ CAP_state;
        // If the current state and the latched state are not
        // the same, there must have been a release. The XOR
        // of the two state variables will output the sensors
        // that were released.
        key_data_release |= current_state ^ latchedState;

        arr_key_data[KEY_PLUS] = latchedState & KEY_PLUS_MASK;
        arr_key_data[KEY_MINUS] = latchedState & KEY_MINUS_MASK;
        arr_key_data[KEY_MOVE] = latchedState & KEY_MOVE_MASK;
        arr_key_data[KEY_SET] = latchedState & KEY_SET_MASK;

        arr_key_press_event[KEY_PLUS]   = key_data_press & KEY_PLUS_MASK;
        arr_key_press_event[KEY_MINUS]  = key_data_press & KEY_MINUS_MASK;
        arr_key_press_event[KEY_SET]    = key_data_press & KEY_SET_MASK;
        arr_key_press_event[KEY_MOVE]   = key_data_press & KEY_MOVE_MASK;

        arr_key_release_event[KEY_PLUS]  = key_data_release & KEY_PLUS_MASK;
        arr_key_release_event[KEY_MINUS] = key_data_release & KEY_MINUS_MASK;
        arr_key_release_event[KEY_SET]   = key_data_release & KEY_SET_MASK;
        arr_key_release_event[KEY_MOVE]  = key_data_release & KEY_MOVE_MASK;

        CAP_state = current_state;

        start_idle_time_count = GetTimerTick();

        for(i = 0; i < MAX_KEY; i++)
        {
            if (arr_key_press_event[i])
            {
                start_hold_time = GetTimerTick();
                break;
            }
        }

        for(i = 0; i < MAX_KEY; i++)
        {
            if (arr_key_data[i])
            {
                if (GetTimerTick() - start_hold_time >= 2000)
                {
                    if (arr_key_hold_event[i] == 0)
                    {
                        arr_key_hold_event[i] = 1;
                    }
                    else if(arr_key_hold_event[i] == 2)
                    {
                        arr_key_on_hold_event[i] = 1;
                    }

                }
                break;
            }
        }

        for(i = 0; i < MAX_KEY; i++)
        {
            if (arr_key_release_event[i])
            {
                arr_key_hold_event[i] = 0;
                arr_key_on_hold_event[i] = 0;
                break;
            }
        }

        if (mode == kIdle)
        {
            BuzzerBeep();
            SetIdleMode(kRun);
        }
        else
        {
            // emit event
            EmitKeyEvent();
        }
    }
    // Check idle time
    if ((GetTimerTick() - start_idle_time_count >= idle_time_ref) && mode == kRun)
    {
        if (enable_auto_off_screen)
        {
            SetIdleMode(kIdle);
        }
    }

    // action when key touch press or hold
    if ((current_key_event % 3) == 2 && current_key_event != NO_KEY_EVENT) // hold
    {
        // replacement the led by buzzer
        //LED_DEG_FAHR(LED_DEG);
        CreateTimer(300, 0, BuzzerOff, NULL);
    }
    else if ((current_key_event % 3) == 0 && current_key_event != NO_KEY_EVENT)
    {
        //LED_DEG_FAHR(LED_FAHR);
        CreateTimer(300, 0, BuzzerOff, NULL);
    }

    // screen process
    ScreenProcess();

    // clear all key event
    if (current_key_event != NO_KEY_EVENT)
    {
        current_key_event = NO_KEY_EVENT;
    }
}



mode_t GetRunMode(void)
{
    return mode;
}

void KeyInterruptCallback(void)
{
    if(I2C_INT_STATUS())
    {
        I2C_ALERT_CLEAR_FLAG();
        key_interrupt = 1;
    }
}
