#pragma once 

#include <stdint.h>
#include "Modbus_Regmap.h"

void Automation_Garage_SaveSettings(ModbusSettings_TypeDef *settings);
int8_t Automation_Garage_CheckSavedSettings();
void Automation_Garage_Sheduler();