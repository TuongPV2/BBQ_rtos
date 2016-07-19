#ifndef _MYUI_H_
#define _MYUI_H_
#include "cap1106.h"

#define MAX_SCREEN  10

#define KEY_SET_PRESS_EVENT         0U
#define KEY_SET_RELEASE_EVENT       1U
#define KEY_SET_HOLD_EVENT          2U

#define KEY_MOVE_PRESS_EVENT         3U
#define KEY_MOVE_RELEASE_EVENT       4U
#define KEY_MOVE_HOLD_EVENT          5U

#define KEY_PLUS_PRESS_EVENT         6U
#define KEY_PLUS_RELEASE_EVENT       7U
#define KEY_PLUS_HOLD_EVENT          8U

#define KEY_MINUS_PRESS_EVENT         9U
#define KEY_MINUS_RELEASE_EVENT       10U
#define KEY_MINUS_HOLD_EVENT          11U

#define KEY_SET_ON_HOLD_EVENT           (KEY_SET_HOLD_EVENT + 20)
#define KEY_MOVE_ON_HOLD_EVENT           (KEY_MOVE_HOLD_EVENT + 20)
#define KEY_PLUS_ON_HOLD_EVENT           (KEY_PLUS_HOLD_EVENT + 20)
#define KEY_MINUS_ON_HOLD_EVENT           (KEY_MINUS_HOLD_EVENT + 20)

#define NO_KEY_EVENT                  255U

typedef uint8_t (*event_hook_handle)(uint8_t event);
typedef void (*key_handle)(uint8_t event);
typedef void (*update_handle)(void* ptr);

typedef enum _mode
{
    kIdle = 0,
    kRun
} mode_t;

typedef struct _screen
{
    // data
    uint8_t screen_id;
    uint8_t notifydatachanged;
    uint8_t key_lock; // enable or disable touch key for this screen
    uint8_t enable_auto_update; // auto call update_screen
    uint8_t status_data; // status of probe
    uint8_t mode_data; // Current or target mode
    uint8_t deg_mode_data; // C F mode
    uint8_t bat_status; // Bat status led
    uint16_t screen_data;
    // function
    update_handle update_screen;
    key_handle key_set_handle;
    key_handle key_move_handle;
    key_handle key_plus_handle;
    key_handle key_minus_handle;
}screen_t;

void UI_Init(void);
void UIProcess(void);
screen_t * GetScreenContext(uint8_t screen_id);
screen_t* GetCurrentScreen(void);
void SetScreenActive(uint8_t _screen_id);
void RegisterScreen(uint8_t _screen_id, update_handle _update_screen, key_handle _key_set_handle,\
     key_handle _key_move_handle, key_handle _key_plus_handle, key_handle _key_minus_handle);
mode_t GetRunMode(void);
void SetIdleMode(mode_t _mode);
void SetEventHookCallback(event_hook_handle handle);
void KeyInterruptCallback(void);
void ScreenSetAutoOff(uint8_t enable);

#endif
