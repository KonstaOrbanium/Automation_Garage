#pragma once 

#include <stdint.h>
#include "mik32_hal_timer32.h"
#include "Modbus_Regmap.h"

void Automation_Garage_SaveSettings(ModbusSettings_TypeDef *settings);
int8_t Automation_Garage_CheckSavedSettings();
void Automation_Garage_Sheduler();
void Automation_Garage_InitAllObjects();
void Automation_Garage_TestProceed();