#include <stdbool.h>
#include <string.h>
#include "mik32_hal.h"
#include "mik32_hal_pcc.h"
#include "mik32_hal_gpio.h"

#include "Modbus_Regmap.h"


void *Modbus_Regmap_Params[NUMBER_OF_OUTPUTS + NUMBER_OF_SETTINGS];
bool Modbus_Regmap_Coils[NUMBER_OF_OUTPUTS] = {0};
uint16_t Modbus_Regmap_Settings[NUMBER_OF_SETTINGS] = {0};


int8_t Modbus_Regmap_InitObject(uint16_t addr, void *pObject) {
    int8_t result = 0;
   
    // if (pObject) {
    //     if (addr <= (NUMBER_OF_OUTPUTS + NUMBER_OF_SETTINGS) * 2) {
    //         Modbus_Regmap_Params[addr] = pObject;
    //         result = 1;
    //     }
    // }

    if (addr >= FOG_LIGHT_ADDR && addr <= TEST_LIGHT) {
        Modbus_Regmap_Coils[addr] = *(bool*)pObject;
    } else if (addr >= FOG_LIGHT_B_MAX_ADDR && addr <= TEST_LIGHT_B_TIME_ADDR) {
        Modbus_Regmap_Settings[addr % FOG_LIGHT_B_MAX_ADDR] = *(uint16_t*)pObject;
    }
    return result;
}

int8_t Modbus_Regmap_GetItem(const uint16_t addr, void *pObject, const uint32_t typeSize) {
    int8_t result = 0;
    if ((void*)Modbus_Regmap_Params[addr]) {
        memcpy(pObject, (void*)Modbus_Regmap_Params[addr], typeSize);
    }
    result = pObject != NULL ? 1 : result;

    return result;
}

int8_t Modbus_Regmap_GetCopyOfItem(const uint16_t addr, void *pObject, const uint32_t typeSize) {
    int8_t result = 0;
    if ((void*)Modbus_Regmap_Params[addr]) {
        memcpy(pObject, (void*)Modbus_Regmap_Params[addr], typeSize);
    }
    result = pObject != NULL ? 1 : result;

    return result;
}

int8_t Modbus_Regmap_SetItem(const uint16_t addr, void *pObject, const uint32_t typeSize) {
    int8_t result = 0;

    memcpy((void*)Modbus_Regmap_Params[addr], pObject, typeSize);
    result = Modbus_Regmap_Params[addr] != NULL ? 1 : result;
    
    return result;
}

