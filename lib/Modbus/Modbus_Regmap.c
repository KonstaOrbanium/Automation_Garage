#include <stdbool.h>
#include <string.h>
#include "mik32_hal.h"
#include "mik32_hal_pcc.h"
#include "mik32_hal_gpio.h"

#include "Modbus_Regmap.h"


void *Modbus_Regmap_Params[NUMBER_OF_OUTPUTS + NUMBER_OF_SETTINGS];
//uint16_t Modbus_Regmap_Settings[NUMBER_OF_SETTINGS] = {0};


int8_t Modbus_Regmap_InitObject(uint8_t addr, void *pObject) {
    int8_t result = 0;
   
    if (pObject) {
        if (addr <= (NUMBER_OF_OUTPUTS + NUMBER_OF_SETTINGS) * 2) {
            Modbus_Regmap_Params[addr] = pObject;
            result = 1;
        }
    }
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

