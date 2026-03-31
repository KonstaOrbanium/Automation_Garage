#include <stdbool.h>
#include <string.h>
#include <math.h>
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

uint32_t userPow(uint32_t a, uint32_t b) {
    const uint32_t c = a;

    for (size_t i = 0; i < b - 1; ++i) {
        a *= c;
    }
    return a;
}

void Automation_Garage_InitAllObjects() {
    for (int i = FOG_LIGHT_ADDR; i <= TEST_LIGHT; ++i) {
        Modbus_Regmap_InitObject(i, (void*)(&MbSettings.fogLight + i));
    }
    for (int i = FOG_LIGHT_B_MAX_ADDR; i <= TEST_LIGHT_B_TIME_ADDR; ++i) {
        Modbus_Regmap_InitObject((i + 7) % FOG_LIGHT_B_MAX_ADDR, (void*)(&MbSettings.fogLightBMax + (i % FOG_LIGHT_B_MAX_ADDR)));
    }
}

void Automation_Garage_SaveSettings(ModbusSettings_TypeDef *settings) {

}

int8_t Automation_Garage_CheckSavedSettings() {
    int8_t result = -1;

    return result;
}


void Automation_Garage_Sheduler() {


}
extern TIMER32_HandleTypeDef htimer32_1;
extern TIMER32_CHANNEL_HandleTypeDef htimer32_channel0;
extern TIMER32_CHANNEL_HandleTypeDef htimer32_channel1;
extern TIMER32_CHANNEL_HandleTypeDef htimer32_channel2;
extern TIMER32_CHANNEL_HandleTypeDef htimer32_channel3;

bool isTestCommand = true;

void Automation_Garage_TestProceed() {
    //static bool isFilledOnce = false;
    int parrot = 0;
    static int coef = 1;
    static int fillFactor = 2;

    // if (!isFilledOnce) {
    //     for (int i = 1; i <= sizeof(gammaCorrection) / sizeof(gammaCorrection[0]) - 1; ++i) {
    //         gammaCorrection[i] = htimer32_1.Top * (float)(powf((float)i / 100., 2.2) * 2.);
    //         isFilledOnce = true;
    //     }
    // }
    
    bool isTestCommand = false;
    Modbus_Regmap_GetCopyOfItem(TEST_LIGHT, &isTestCommand, sizeof(bool));
    if (isTestCommand) {
        if (fillFactor >= 99) {
            coef = -1;     
        }
        if (fillFactor <= 1) {
            coef = 1;
            isTestCommand = false;
            Modbus_Regmap_SetItem(TEST_LIGHT, &isTestCommand, sizeof(bool));
        }
        //parrot = htimer32_1.Top * fillFactor / 50;
       // parrot = htimer32_1.Top * (float)(powf((float)fillFactor / 100., 2.2) * 2.);
        parrot = htimer32_1.Top * userPow(fillFactor, 2) / 5000;
        //parrot = gammaCorrection[fillFactor <= 1 ? 1 : fillFactor - 1];
        
       // HAL_Timer32_Top_Set(&htimer32_1, htimer32_1.Top);
        HAL_Timer32_Channel_OCR_Set(&htimer32_channel0, (parrot - 1) >> 1);
        HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, (parrot - 1) >> 1);
        HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, (parrot - 1) >> 1);
        HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, (parrot - 1) >> 1);
        
        fillFactor += coef;
    }
    
}
