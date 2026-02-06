#include <stdbool.h>
#include <string.h>
#include "mik32_hal.h"
#include "mik32_hal_pcc.h"
#include "mik32_hal_gpio.h"

#include "Modbus_Regmap.h"


void *Modbus_Regmap_Params[NUMBER_OF_OUTPUTS + NUMBER_OF_SETTINGS] = { 0 };
//uint16_t Modbus_Regmap_Settings[NUMBER_OF_SETTINGS] = {0};


int8_t Modbus_Regmap_GetCopyOfItem(const uint16_t addr, void *pObject, const uint32_t typeSize) {
    int8_t result = -1;
    memcpy(pObject, (void*)&Modbus_Regmap_Params[addr], typeSize);
    result = pObject != NULL ? 0 : result;

    return result;
}

int8_t Modbus_Regmap_SetItem(const uint16_t addr, void *pObject, const uint32_t typeSize) {
    int8_t result = -1;
    memcpy((void*)&Modbus_Regmap_Params[addr], pObject, typeSize);
    result = Modbus_Regmap_Params[addr] != NULL ? 0 : result;
    
    return result;
}

