#ifndef  MENU_H
#define MENU_H

#include "myLED.h"
#include "myUI.h"

#define ROOT_SREEN_ID       1U
#define TARGET_SCREEN_ID    2U

void Menu_Init(void);
uint8_t GetProbePlugStatus(void);

// Root screen
void RootScreenUpdate(void* pdata);
void RootScreenKeySetHandle(uint8_t event);
void RootScreenKeyMoveHandle(uint8_t event);
void RootScreenKeyMinusHandle(uint8_t event);
void RootScreenKeyPlusHandle(uint8_t event);

// Target screen
void TargetScreenUpdate(void* pdata);
void TargetScreenKeySetHandle(uint8_t event);
void TargetScreenKeyMoveHandle(uint8_t event);
void TargetScreenKeyMinusHandle(uint8_t event);
void TargetScreenKeyPlusHandle(uint8_t event);

//

#endif
