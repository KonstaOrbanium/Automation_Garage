#pragma once 

#include "mik32_hal_gpio.h"

/* MB FC 1 READ 5 WRITE */
#define FOG_LIGHT_ADDR          0   ///< Противотуманное освещение 
#define STOP_LIGHT              1   ///< Стоп-сигнал
#define TURN_LIGHT              2   ///< Указатель поворота (Включить в режиме стандартный)
#define REVERSING_LIGHT         3   ///< Задний ход  
#define PARKING_LIGHT           4   ///< Габарит
#define STOP_LIGHT_STROBE       5   ///< Стоп-сигнал (Включить в режиме стробоскоп)
#define TEST_LIGHT              6   ///< Тест 


/* MB FC 3 READ 6 WRITE */
#define FOG_LIGHT_B_MAX_ADDR         0x1F4   ///< Противотуманное освещение (Максимальная яркость)
#define STOP_LIGHT_B_MAX_ADDR        0x1F5   ///< Стоп-сигнал (Максимальная яркость)
#define TURN_LIGHT_B_MAX_ADDR        0x1F6   ///< Указатель поворота (Максимальная яркость)
#define REVERSING_LIGHT_B_MAX_ADDR   0x1F7   ///< Задний ход (Максимальная яркость)
#define PARKING_LIGHT_B_MAX_ADDR     0x1F8   ///< Габарит (Максимальная яркость)
#define STOP_LIGHT_STROBE_TIME_ADDR  0x1F9   ///< Стоп-сигнал (Время режима стробоскоп)
#define TURNING_LIGHT_MODE_ADDR      0x1FA   ///< Указатель поворота (Режим)
#define PARKING_LIGHT_B_TIME_ADDR    0x1FB   ///< Габарит (Продолжительность изменения яркости)
#define TEST_LIGHT_B_TIME_ADDR       0x1FC   ///< Режим тест (Продолжительность изменения яркости)

#define NUMBER_OF_OUTPUTS       7
#define NUMBER_OF_SETTINGS      9

#define USART1_REDE_PIN GPIO_PIN_11
#define USART1_REDE_PORT GPIO_1



typedef struct {
    volatile uint16_t fogLight;               ///< Противотуманное освещение 
    volatile uint16_t stopLight;               ///< Стоп-сигнал
    volatile uint16_t turnLight;               ///< Указатель поворота (Включить в режиме стандартный)
    volatile uint16_t reversingLight;          ///< Задний ход  
    volatile uint16_t parkingLight;            ///< Габарит
    volatile uint16_t stopLightStrobe;         ///< Стоп-сигнал (Включить в режиме стробоскоп)
    volatile uint16_t testLight;               ///< Тест 

    volatile uint16_t fogLightBMax;            ///< Противотуманное освещение (Максимальная яркость)
    volatile uint16_t stopLightBMax;           ///< Стоп-сигнал (Максимальная яркость)
    volatile uint16_t turnLightBMax;           ///< Указатель поворота (Максимальная яркость)
    volatile uint16_t reversingLightBMax;      ///< Задний ход (Максимальная яркость)
    volatile uint16_t parkingLightBMax;        ///< Габарит (Максимальная яркость)
    volatile uint16_t stopLightStrobeTime;     ///< Стоп-сигнал (Время режима стробоскоп)
    volatile uint16_t turningLightMode;        ///< Указатель поворота (Режим)
    volatile uint16_t parkingLightBTime;       ///< Габарит (Продолжительность изменения яркости)
    volatile uint16_t testLightBTime;          ///< Режим тест (Продолжительность изменения яркости)

}ModbusSettings_TypeDef;

typedef union {
    uint16_t u16[2];
	uint32_t u32;
	float f;
} Modbus_Regmap_UTOF_TypeDef;

extern void *Modbus_Regmap_DoStates[NUMBER_OF_OUTPUTS + NUMBER_OF_SETTINGS];

 
int8_t Modbus_Regmap_GetCopyOfItem(const uint16_t addr, void  *value, const uint32_t typeSize);
int8_t Modbus_Regmap_SetItem(const uint16_t addr, void *pObject, const uint32_t typeSize);