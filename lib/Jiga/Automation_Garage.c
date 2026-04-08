#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "mik32_hal.h"
#include "mik32_hal_pcc.h"
#include "mik32_hal_gpio.h"


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
    .fogLightBMax        = 99,         ///< Противотуманное освещение (Максимальная яркость)
    .stopLightBMax       = 99,         ///< Стоп-сигнал (Максимальная яркость)
    .turnLightBMax       = 99,         ///< Указатель поворота (Максимальная яркость)
    .reversingLightBMax  = 99,         ///< Задний ход (Максимальная яркость)
    .parkingLightBMax    = 99,          ///< Габарит (Максимальная яркость)
    .stopLightStrobeTime = 5,           ///< Стоп-сигнал (Время режима стробоскоп)
    .turningLightMode    = 1,           ///< Указатель поворота (Режим)
    .parkingLightBTime   = 5,           ///< Габарит (Продолжительность изменения яркости)
    .testLightBTime      = 5,           ///< Режим тест (Продолжительность изменения яркости)
};




extern TIMER32_HandleTypeDef htimer32_1;
extern TIMER32_CHANNEL_HandleTypeDef htimer32_channel0; ///< White
extern TIMER32_CHANNEL_HandleTypeDef htimer32_channel1; ///< Red (parking and fog)
extern TIMER32_CHANNEL_HandleTypeDef htimer32_channel2; ///< Red (stop)
extern TIMER32_CHANNEL_HandleTypeDef htimer32_channel3; ///< Yellow
extern bool Modbus_Regmap_Coils[NUMBER_OF_OUTPUTS];
extern uint16_t Modbus_Regmap_Settings[NUMBER_OF_SETTINGS];

bool isTestCommand = true;
uint32_t gammaCorrection[99] = { 0 };

uint16_t maxChannelsBrightness[] = { 99, 99, 99, 99 };
TIMER32_CHANNEL_HandleTypeDef *timChannelMas[4] = { &htimer32_channel0, &htimer32_channel1, &htimer32_channel2, &htimer32_channel3};

uint32_t userPow(uint32_t a, uint32_t b) {
    uint32_t c = a;

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
        Modbus_Regmap_InitObject(i, (void*)(&MbSettings.fogLightBMax + (i % FOG_LIGHT_B_MAX_ADDR)));
    }
    for (int i = 1; i <= sizeof(gammaCorrection) / sizeof(gammaCorrection[0]) - 1; ++i) {
        gammaCorrection[i] = htimer32_1.Top * userPow(i, 2) / 5000;          
    }
}

void Automation_Garage_SaveSettings(ModbusSettings_TypeDef *settings) {

}

int8_t Automation_Garage_CheckSavedSettings() {
    int8_t result = -1;

    return result;
}

void Automation_Garage_TestProceed() {
    int parrot = 0;
    static int coef[] = { 1, 1, 1, 1 };
    static uint16_t fillFactor[] = { 2, 2, 2, 2 };
    static bool isFillDone[4] = { false, };
    const uint16_t minChannelsBrightness = 1;
    static uint16_t currentIndex = 0;
    bool isTestCommand = false;
    
    //Modbus_Regmap_GetCopyOfItem(TEST_LIGHT, &isTestCommand, sizeof(bool));
    isTestCommand = Modbus_Regmap_Coils[TEST_LIGHT];
     if (isTestCommand) {
       
        if (!isFillDone[currentIndex]) {
            //Modbus_Regmap_GetCopyOfItem(currentIndex + offset, &maxChannelsBrightness[currentIndex], sizeof(uint16_t));
            if (fillFactor[currentIndex] >= Modbus_Regmap_Settings[currentIndex] - 1) {
                coef[currentIndex] = -1;     
            }
            if (fillFactor[currentIndex] <= minChannelsBrightness) {
                coef[currentIndex] = 1;
                isFillDone[currentIndex] = true;
                if (isFillDone[0] && isFillDone[1] && isFillDone[2] && isFillDone[3]) {
                    isTestCommand = false;
                    //Modbus_Regmap_SetItem(TEST_LIGHT, &isTestCommand, sizeof(bool));
                    Modbus_Regmap_Coils[TEST_LIGHT] = isTestCommand;
                    for (uint8_t i = 0; i < 4; ++i) {
                        isFillDone[i] = false;
                    }
                }
            }
           // parrot = htimer32_1.Top * userPow(fillFactor[currentIndex], 2) / 5000;
            parrot = gammaCorrection[fillFactor[currentIndex]];
            HAL_Timer32_Channel_OCR_Set(timChannelMas[currentIndex], (parrot - 1) >> 1);
            fillFactor[currentIndex] += coef[currentIndex];
        }      
        currentIndex++;
        currentIndex = currentIndex >= 4 ? 0 : currentIndex; 
    }
   
}

void Automation_Garage_SetBrightness() {
    static uint16_t oldValue = 0;
    const uint8_t offset = 7U;     ///< Смещение старта нужных адресов фонарей в массиве
    static uint8_t currentIndex = 0;
   
    //Modbus_Regmap_GetCopyOfItem(currentIndex + offset, &oldValue, sizeof(uint16_t));
    oldValue = Modbus_Regmap_Settings[currentIndex];       
    
    if (oldValue >= 99) {
        oldValue = 99;
    }
    if (oldValue <= 1) {
        oldValue = 1;
    }
    //Modbus_Regmap_SetItem(currentIndex + offset, &oldValue, sizeof(uint16_t));
    Modbus_Regmap_Settings[currentIndex] = oldValue;
    currentIndex++;
    currentIndex = currentIndex >= 4 ? 0 : currentIndex; 
}


void Automation_Garage_SetStopLight() {
    static uint32_t timeCounter = 0;
    bool defaultValue = false;
    bool strobeValue = false;
    static uint32_t defaultTimeValue = 5;
    uint16_t timeValue = 0;
    uint32_t parrot = 0;
    uint16_t stopLightMaxBrightness = 0;
    static uint32_t tickSave = 0;
    static bool isTimeSaved = false;

    // Modbus_Regmap_GetCopyOfItem(STOP_LIGHT, &defaultValue, sizeof(bool));
    // Modbus_Regmap_GetCopyOfItem(STOP_LIGHT_STROBE, &strobeValue, sizeof(bool));
    // Modbus_Regmap_GetCopyOfItem(STOP_LIGHT_STROBE_TIME_ADDR, &timeValue, sizeof(uint16_t));
    // Modbus_Regmap_GetCopyOfItem(STOP_LIGHT_B_MAX_ADDR, &stopLightMaxBrightness, sizeof(uint16_t));
    
    
    timeValue = Modbus_Regmap_Settings[STOP_LIGHT_STROBE_TIME_ADDR % FOG_LIGHT_B_MAX_ADDR];
    if (timeValue != defaultTimeValue) {
        defaultTimeValue = timeValue;
    }
    defaultValue = Modbus_Regmap_Coils[STOP_LIGHT];
    strobeValue  = Modbus_Regmap_Coils[STOP_LIGHT_STROBE];
    if (defaultValue && !strobeValue) {
        stopLightMaxBrightness  = Modbus_Regmap_Settings[STOP_LIGHT_B_MAX_ADDR % FOG_LIGHT_B_MAX_ADDR];
        parrot = htimer32_1.Top * userPow(stopLightMaxBrightness, 2) / 5000;
        HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, (parrot - 1) >> 1);
    } else if (strobeValue) {
        if (!isTimeSaved) {
            tickSave = timeCounter;
            isTimeSaved = true;
        }
        if (timeCounter - tickSave >= (timeValue * 2)) {
            //Modbus_Regmap_SetItem(STOP_LIGHT_STROBE, &strobeValue, sizeof(bool));  
            HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, (parrot - 1) >> 1);
        } else {
            if (!(timeCounter & 0x01)) {
            parrot = htimer32_1.Top * userPow(stopLightMaxBrightness, 2) / 5000;
            HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, (parrot - 1) >> 1);
        } else {
            HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, 0 >> 1);
        }  
        }
        
    } else {
        HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, 0 >> 1);
        isTimeSaved = false; 
    }
    timeCounter++;
}


void setTurnPointer() {
    bool turnDefaultValue = false;
    uint16_t turnModeValue = 0;
    uint32_t parrot = 0;
    uint16_t turnLightMaxBrightness = 0;
    static uint32_t secCounter = 0;  ///< 10 тиков - 1 секунда
    static bool isRevertState = false;
    static uint32_t fillFactor = 2;
    static bool isFillDone = false;
    static uint32_t timeSave = 0;

    Modbus_Regmap_GetCopyOfItem(TURN_LIGHT, &turnDefaultValue, sizeof(bool));
    Modbus_Regmap_GetCopyOfItem(TURNING_LIGHT_MODE_ADDR, &turnModeValue, sizeof(uint16_t));
    Modbus_Regmap_GetCopyOfItem(TURN_LIGHT_B_MAX_ADDR, &turnLightMaxBrightness, sizeof(uint16_t));
    
    if (turnDefaultValue && !turnModeValue) {
        if (!(secCounter % 20)) {
            parrot = htimer32_1.Top * userPow(turnLightMaxBrightness, 2) / 5000;
            if (!isRevertState) {
                HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, (parrot - 1) >> 1);
                isRevertState = true;
            } else {
                HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, 0 >> 1);
                isRevertState = false;
            }        
        }
    } else if (turnDefaultValue && turnModeValue) {
        if (secCounter - timeSave >= 20 && isFillDone) {
            isFillDone = false;  
        } else if (!isFillDone) {
            if (fillFactor >= 99) {
                fillFactor = 2;
                isFillDone = true;
                timeSave = secCounter;
                HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, 0 >> 1);
            }
            parrot = htimer32_1.Top * userPow(fillFactor, 2) / 5000;
            HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, (parrot - 1) >> 1);
        }
    } else {
        HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, 0 >> 1);
    }
    
    secCounter++;
    fillFactor++;
}

void setFogLight() {

}

