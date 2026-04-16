#pragma once 

#include <stdint.h>
#include "mik32_hal_timer32.h"
#include "mik32_hal_wdt.h"
#include "mik32_hal_eeprom.h"
#include "Modbus_Regmap.h"
#include "Modbus.h"

#pragma pack(push, 4)

typedef struct {
    volatile bool fogLight;                    ///< Противотуманное освещение 
    volatile bool stopLight;                   ///< Стоп-сигнал
    volatile bool turnLight;                   ///< Указатель поворота (Включить в режиме стандартный)
    volatile bool reversingLight;              ///< Задний ход  
    volatile bool parkingLight;                ///< Габарит
    volatile bool stopLightStrobe;             ///< Стоп-сигнал (Включить в режиме стробоскоп)
    volatile bool testLight;                   ///< Тест 
    volatile bool res;                         ///< Reserved 

    volatile uint16_t fogLightBMax;            ///< Противотуманное освещение (Максимальная яркость)
    volatile uint16_t stopLightBMax;           ///< Стоп-сигнал (Максимальная яркость)
    volatile uint16_t turnLightBMax;           ///< Указатель поворота (Максимальная яркость)
    volatile uint16_t reversingLightBMax;      ///< Задний ход (Максимальная яркость)
    volatile uint16_t parkingLightBMax;        ///< Габарит (Максимальная яркость)
    volatile uint16_t stopLightStrobeTime;     ///< Стоп-сигнал (Время режима стробоскоп)
    volatile uint16_t turningLightMode;        ///< Указатель поворота (Режим)
    volatile uint16_t parkingLightBTime;       ///< Габарит (Продолжительность изменения яркости)
    volatile uint16_t testLightBTime;          ///< Режим тест (Продолжительность изменения яркости)
    volatile uint16_t saveCommand;             ///< Сохранене параметров
    volatile uint16_t flashErase;              ///< Очистка флэш (eeprom)

    volatile uint16_t crc;
}Automation_Garage_StoredSettings_Typedef;

#pragma pack(pop)


extern Automation_Garage_StoredSettings_Typedef storedSettings;

void Automation_Garage_SaveSettings();
void Automation_Garage_FlashErase();
void Auromation_Garage_InitDefaultSettings();
void Automation_Garage_Sheduler();
void Automation_Garage_TestProceed();
void Automation_Garage_SetBrightness();
void Automation_Garage_SetStopLight();
void setTurnPointer();
void setReverseLight();
void setFogLight();