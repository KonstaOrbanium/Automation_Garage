#include <stdbool.h>
#include <string.h>
#include "mik32_hal.h"
#include "mik32_hal_pcc.h"
#include "mik32_hal_gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "Automation_Garage.h"

ModbusSettings_TypeDef MbSettings = {
    /* DO */
    .fogLight            = 0,           ///< Противотуманное освещение 
    .stopLight           = 0,           ///< Стоп-сигнал
    .turnLight           = 0,           ///< Указатель поворота (Включить в режиме стандартный)
    .reversingLight      = 0,           ///< Задний ход  
    .parkingLight        = 0,           ///< Габарит
    .stopLightStrobe     = 0,           ///< Стоп-сигнал (Включить в режиме стробоскоп)
    .testLight           = 0, 

    /* Parameters */
    .fogLightBMax        = 100,         ///< Противотуманное освещение (Максимальная яркость)
    .stopLightBMax       = 100,         ///< Стоп-сигнал (Максимальная яркость)
    .turnLightBMax       = 100,         ///< Указатель поворота (Максимальная яркость)
    .reversingLightBMax  = 100,         ///< Задний ход (Максимальная яркость)
    .parkingLightBMax    = 50,          ///< Габарит (Максимальная яркость)
    .stopLightStrobeTime = 5,           ///< Стоп-сигнал (Время режима стробоскоп)
    .turningLightMode    = 1,           ///< Указатель поворота (Режим)
    .parkingLightBTime   = 5,           ///< Габарит (Продолжительность изменения яркости)
    .testLightBTime      = 5,           ///< Режим тест (Продолжительность изменения яркости)
};


void Automation_Garage_InitAllObjects() {
    for (int i = FOG_LIGHT_ADDR; i <= TEST_LIGHT; ++i) {
        Modbus_Regmap_SetItem(i, (void*)(&MbSettings.fogLight + i), sizeof(bool));
    }
    for (int i = FOG_LIGHT_B_MAX_ADDR; i <= TEST_LIGHT_B_TIME_ADDR; ++i) {
        Modbus_Regmap_SetItem(i, (void*)(&MbSettings.fogLight + i), sizeof(uint16_t));
    }
}

void Automation_Garage_SaveSettings(ModbusSettings_TypeDef *settings) {

}

int8_t Automation_Garage_CheckSavedSettings() {

}


void Automation_Garage_Sheduler() {


}

void Automation_Garage_TestTaskProceed() {
    

}
