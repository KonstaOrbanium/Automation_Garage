#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "mik32_hal.h"
#include "mik32_hal_pcc.h"
#include "mik32_hal_gpio.h"
#include "mik32_hal_spifi_w25.h"

#include "Automation_Garage.h"

int8_t Automation_Garage_CheckSavedSettings(Automation_Garage_StoredSettings_Typedef *pdefaultSettings, uint32_t size);
void Automation_Garage_InitAllObjects(Automation_Garage_StoredSettings_Typedef *pdefaultSettings);

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
    .parkingLightBMax    = 50,          ///< Габарит (Максимальная яркость)
    .stopLightStrobeTime = 5,           ///< Стоп-сигнал (Время режима стробоскоп)
    .turningLightMode    = 1,           ///< Указатель поворота (Режим)
    .parkingLightBTime   = 2,           ///< Габарит (Продолжительность изменения яркости)
    .testLightBTime      = 4,           ///< Режим тест (Продолжительность изменения яркости)
};




extern TIMER32_HandleTypeDef htimer32_1;
extern TIMER32_CHANNEL_HandleTypeDef htimer32_channel0; ///< White
extern TIMER32_CHANNEL_HandleTypeDef htimer32_channel1; ///< Red (parking and fog)
extern TIMER32_CHANNEL_HandleTypeDef htimer32_channel2; ///< Red (stop)
extern TIMER32_CHANNEL_HandleTypeDef htimer32_channel3; ///< Yellow
extern bool Modbus_Regmap_Coils[NUMBER_OF_OUTPUTS];
extern uint16_t Modbus_Regmap_Settings[NUMBER_OF_SETTINGS];
extern WDT_HandleTypeDef hwdt;

bool isTestCommand = true;
volatile const uint32_t gammaCorrection[] = { 5, 5, 6, 6, 7, 7, 8, 9, 10, 10, 11, 12, 13, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 27, 28, 29, 30, 31, 33, 34, 35, 37, 38, 40, 41, 43, 44, 46, 
47, 49, 50, 52, 54, 55, 57, 59, 60, 62, 64, 66, 68, 70, 71, 73, 75, 77, 79, 81, 83, 86, 88, 90, 92, 94, 96, 99, 101, 103, 105, 108, 110, 113, 115, 117, 120, 122, 125, 127, 
130, 133, 135, 138, 141, 143, 146, 149, 152, 154, 157, 160, 163, 166, 169, 172, 175, 178, 181, 184, 187, 190, 193, 196, 199, 203, 206, 209, 212, 216, 219, 222, 226, 229, 233, 236, 240, 243, 247, 250, 
254, 258, 261, 265, 269, 272, 276, 280, 284, 287, 291, 295, 299, 303, 307, 311, 315, 319, 323, 327, 331, 335, 340, 344, 348, 352, 356, 361, 365, 369, 374, 378, 383, 387, 391, 396, 400, 405, 410, 414, 
419, 423, 428, 433, 438, 442, 447, 452, 457, 462, 466, 471, 476, 481, 486, 491, 496, 501, 506, 511, 517, 522, 527, 532, 537, 543, 548, 553, 559, 564, 569, 575, 580, 586, 591, 597, 602, 608, 613, 619, 
625, 630, 636, 642, 647, 653, 659, 665, 671, 677, 682, 688, 694, 700, 706, 712, 718, 724, 731, 737, 743, 749, 755, 761, 768, 774, 780, 787, 793, 799, 806, 812, 819, 825, 832, 838, 845, 851, 858, 865, 
871, 878, 885, 891, 898, 905, 912, 919, 926, 932, 939, 946, 953, 960, 967, 974, 981, 989, 996, 1003, 1010, 1017, 1024, 1032, 1039, 1046, 1054, 1061, 1068, 1076, 1083, 1091, 1098, 1106, 1113, 1121, 1128, 1136, 1144, 1151, 
1159, 1167, 1174, 1182, 1190, 1198, 1206, 1214, 1221, 1229, 1237, 1245, 1253, 1261, 1269, 1277, 1286, 1294, 1302, 1310, 1318, 1326, 1335, 1343, 1351, 1360, 1368, 1376, 1385, 1393, 1402, 1410, 1419, 1427, 1436, 1444, 1453, 1462, 1470, 1479, 
1488, 1496, 1505, 1514, 1523, 1532, 1540, 1549, 1558, 1567, 1576, 1585, 1594, 1603, 1612, 1621, 1631, 1640, 1649, 1658, 1667, 1677, 1686, 1695, 1705, 1714, 1723, 1733, 1742, 1752, 1761, 1771, 1780, 1790, 1799, 1809, 1818, 1828, 1838, 1848, 
1857, 1867, 1877, 1887, 1896, 1906, 1916, 1926, 1936, 1946, 1956, 1966, 1976, 1986, 1996, 2006, 2017, 2027, 2037, 2047, 2057, 2068, 2078, 2088, 2099, 2109, 2119, 2130, 2140, 2151, 2161, 2172, 2182, 2193, 2204, 2214, 2225, 2236, 2246, 2257, 
2268, 2279, 2289, 2300, 2311, 2322, 2333, 2344, 2355, 2366, 2377, 2388, 2399, 2410, 2421, 2432, 2444, 2455, 2466, 2477, 2488, 2500, 2511, 2522, 2534, 2545, 2557, 2568, 2580, 2591, 2603, 2614, 2626, 2637, 2649, 2661, 2672, 2684, 2696, 2708, 
2719, 2731, 2743, 2755, 2767, 2779, 2791, 2803, 2815, 2827, 2839, 2851, 2863, 2875, 2887, 2899, 2911, 2924, 2936, 2948, 2960, 2973, 2985, 2997, 3010, 3022, 3035, 3047, 3060, 3072, 3085, 3097, 3110, 3123, 3135, 3148, 3161, 3173, 3186, 3199, 
3212, 3225, 3237, 3250, 3263, 3276, 3289, 3302, 3315, 3328, 3341, 3354, 3368, 3381, 3394, 3407, 3420, 3433, 3447, 3460, 3473, 3487, 3500, 3513, 3527, 3540, 3554, 3567, 3581, 3594, 3608, 3622, 3635, 3649, 3663, 3676, 3690, 3704, 3718, 3731, 
3745, 3759, 3773, 3787, 3801, 3815, 3829, 3843, 3857, 3871, 3885, 3899, 3913, 3927, 3942, 3956, 3970, 3984, 3999, 4013, 4027, 4042, 4056, 4070, 4085, 4099, 4114, 4128, 4143, 4158, 4172, 4187, 4201, 4216, 4231, 4246, 4260, 4275, 4290, 4305, 
4320, 4334, 4349, 4364, 4379, 4394, 4409, 4424, 4439, 4454, 4470, 4485, 4500, 4515, 4530, 4546, 4561, 4576, 4591, 4607, 4622, 4637, 4653, 4668, 4684, 4699, 4715, 4730, 4746, 4762, 4777, 4793, 4809, 4824, 4840, 4856, 4871, 4887, 4903, 4919, 
4935, 4951, 4967, 4983, 4999, 5015, 5031, 5047, 5063, 5079, 5095, 5111, 5127, 5144, 5160, 5176, 5192, 5209, 5225, 5241, 5258, 5274, 5291, 5307, 5324, 5340, 5357, 5373, 5390, 5407, 5423, 5440, 5457, 5473, 5490, 5507, 5524, 5541, 5557, 5574, 
5591, 5608, 5625, 5642, 5659, 5676, 5693, 5710, 5727, 5744, 5762, 5779, 5796, 5813, 5831, 5848, 5865, 5882, 5900, 5917, 5935, 5952, 5970, 5987, 6005, 6022, 6040, 6057, 6075, 6093, 6110, 6128, 6146, 6163, 6181, 6199, 6217, 6235, 6253, 6270, 
6288, 6306, 6324, 6342, 6360, 6378, 6396, 6415, 6433, 6451, 6469, 6487, 6506, 6524, 6542, 6560, 6579, 6597, 6615, 6634, 6652, 6671, 6689, 6708, 6726, 6745, 6764, 6782, 6801, 6819, 6838, 6857, 6876, 6894, 6913, 6932, 6951, 6970, 6989, 7008, 
7027, 7046, 7065, 7084, 7103, 7122, 7141, 7160, 7179, 7198, 7217, 7237, 7256, 7275, 7295, 7314, 7333, 7353, 7372, 7392, 7411, 7430, 7450, 7470, 7489, 7509, 7528, 7548, 7568, 7587, 7607, 7627, 7647, 7666, 7686, 7706, 7726, 7746, 7766, 7786, 
7806, 7826, 7846, 7866, 7886, 7906, 7926, 7946, 7966, 7987, 8007, 8027, 8047, 8068, 8088, 8108, 8129, 8149, 8170, 8190, 8211, 8231, 8252, 8272, 8293, 8313, 8334, 8355, 8375, 8396, 8417, 8438, 8458, 8479, 8500, 8521, 8542, 8563, 8584, 8605, 
8626, 8647, 8668, 8689, 8710, 8731, 8752, 8773, 8795, 8816, 8837, 8858, 8880, 8901, 8922, 8944, 8965, 8987, 9008, 9030, 9051, 9073, 9094, 9116, 9137, 9159, 9181, 9202, 9224, 9246, 9268, 9289, 9311, 9333, 9355, 9377, 9399, 9421, 9443, 9465, 
9487, 9509, 9531, 9553, 9575, 9597, 9619, 9642, 9664, 9686, 9708, 9731, 9753, 9775, 9798, 9820, 9843, 9865, 9888, 9910, 9933, 9955, 9978, 10000, 10023, 10046, 10068, 10091, 10114, 10137, 10159, 10182, 10205, 10228, 10251, 10274, 10297, 10320, 10343, 10366, 
10389, 10412, 10435, 10458, 10481, 10504, 10528, 10551, 10574, 10597, 10621, 10644, 10667, 10691, 10714, 10738, 10761, 10784, 10808, 10832, 10855, 10879, 10902, 10926, 10950, 10973, 10997, 11021, 11044, 11068, 11092, 11116, 11140, 11164, 11188, 11212, 11236, 11260, 11284, 11308, 
11332, 11356, 11380, 11404, 11428, 11452, 11477, 11501, 11525, 11549, 11574, 11598, 11623, 11647, 11671, 11696, 11720, 11745, 11769, 11794, 11819, 11843, 11868, 11892, 11917, 11942, 11967, 11991, 12016, 12041, 12066, 12091, 12116, 12140, 12165, 12190, 12215, 12240, 12265, 12290, 
12316, 12341, 12366, 12391, 12416, 12441, 12467, 12492, 12517, 12543, 12568, 12593, 12619, 12644, 12670, 12695, 12721, 12746, 12772, 12797, 12823, 12848, 12874, 12900, 12926, 12951, 12977, 13003, 13029, 13054, 13080, 13106, 13132, 13158, 13184, 13210, 13236, 13262, 13288,  };

uint16_t maxChannelsBrightness[] = { 99, 99, 99, 99 };
TIMER32_CHANNEL_HandleTypeDef *timChannelMas[4] = { &htimer32_channel2, &htimer32_channel1, &htimer32_channel0, &htimer32_channel3};

uint32_t userPow(uint32_t a, uint32_t b) {
    uint32_t c = a;

    for (size_t i = 0; i < b - 1; ++i) {
        a *= c;
    }
    return a;
}

void Automation_Garage_InitAllObjects(Automation_Garage_StoredSettings_Typedef *pdefaultSettings) {
    for (int i = FOG_LIGHT_ADDR; i <= TEST_LIGHT; ++i) {
        Modbus_Regmap_InitObject(i, (void*)(&pdefaultSettings->fogLight + i));
    }
    for (int i = FOG_LIGHT_B_MAX_ADDR; i <= TEST_LIGHT_B_TIME_ADDR; ++i) {
        Modbus_Regmap_InitObject(i, (void*)(&pdefaultSettings->fogLightBMax + (i % FOG_LIGHT_B_MAX_ADDR)));
    }
    // float i = 2.;
    // int j = 0;

    // for ( ; j < 1000; i += 0.1, j++) {
    //     gammaCorrection[j] = htimer32_1.Top * powf(i, 2.) / 5000UL;          
    // }
    // for ( ; j < 100;  j++) {
    //     gammaCorrection[j] = htimer32_1.Top * userPow(j, 2) / 5000UL;          
    // }
}


extern HAL_EEPROM_HandleTypeDef heeprom;


void Automation_Garage_SaveSettings() {
    bool isSaveSettings = false;
    isSaveSettings = Modbus_Regmap_Settings[SAVE_SETTINGS % FOG_LIGHT_B_MAX_ADDR];
    Automation_Garage_StoredSettings_Typedef storeSettings = { 0 };
    if (isSaveSettings) {
        const uint8_t pageWords = 32;
        const uint8_t pageCount = 8;
        isSaveSettings = false;
        Modbus_Regmap_Settings[SAVE_SETTINGS % FOG_LIGHT_B_MAX_ADDR] = isSaveSettings;
        for (size_t i = 0; i < NUMBER_OF_OUTPUTS - 1; ++i) {
            *(&storeSettings.fogLight + i) = Modbus_Regmap_Coils[i];
        }
        for (size_t i = 0; i < NUMBER_OF_SETTINGS - 2; ++i) {
            *(&storeSettings.fogLightBMax + i) = Modbus_Regmap_Settings[i];
        }    
        storeSettings.crc = calcCRC((uint8_t*)&storeSettings, sizeof(Automation_Garage_StoredSettings_Typedef) - 2);
        HAL_EEPROM_Erase(&heeprom, 0, pageWords, HAL_EEPROM_WRITE_ALL, 100000);
        HAL_EEPROM_Write(&heeprom, 0, (uint32_t*)&storeSettings, pageCount, HAL_EEPROM_WRITE_ALL, 100000);
    }
}

void Automation_Garage_FlashErase() {
    bool isFlashEraseCommand = false;

    isFlashEraseCommand = Modbus_Regmap_Settings[FLASH_ERASE_COMMAND % FOG_LIGHT_B_MAX_ADDR];

    if (isFlashEraseCommand) {
        isFlashEraseCommand = false;
        Modbus_Regmap_Settings[FLASH_ERASE_COMMAND % FOG_LIGHT_B_MAX_ADDR] = isFlashEraseCommand;
        HAL_EEPROM_Erase(&heeprom, 0, 32, HAL_EEPROM_WRITE_ALL, 100000);
    }
    
}


Automation_Garage_StoredSettings_Typedef storedSettings = {
        /* DO */
        .fogLight            = 0,           ///< Противотуманное освещение 
        .stopLight           = 0,           ///< Стоп-сигнал
        .turnLight           = 0,           ///< Указатель поворота (Включить в режиме стандартный)
        .reversingLight      = 0,           ///< Задний ход  
        .parkingLight        = 0,           ///< Габарит
        .stopLightStrobe     = 0,           ///< Стоп-сигнал (Включить в режиме стробоскоп)
        .testLight           = 0, 
        .res                 = 0,

        /* Parameters */
        .fogLightBMax        = 99,         ///< Противотуманное освещение (Максимальная яркость)
        .stopLightBMax       = 99,         ///< Стоп-сигнал (Максимальная яркость)
        .turnLightBMax       = 99,         ///< Указатель поворота (Максимальная яркость)
        .reversingLightBMax  = 99,         ///< Задний ход (Максимальная яркость)
        .parkingLightBMax    = 50,          ///< Габарит (Максимальная яркость)
        .stopLightStrobeTime = 5,           ///< Стоп-сигнал (Время режима стробоскоп)
        .turningLightMode    = 1,           ///< Указатель поворота (Режим)
        .parkingLightBTime   = 2,           ///< Габарит (Продолжительность изменения яркости)
        .testLightBTime      = 4,           ///< Режим тест (Продолжительность изменения яркости)
        .saveCommand         = 0,
        .flashErase          = 0,
        .crc                 = 0,
    };
void Auromation_Garage_InitDefaultSettings() {
    Automation_Garage_CheckSavedSettings(&storedSettings, sizeof(Automation_Garage_StoredSettings_Typedef));
    Automation_Garage_InitAllObjects(&storedSettings);
}

int8_t Automation_Garage_CheckSavedSettings(Automation_Garage_StoredSettings_Typedef *pdefaultSettings, uint32_t size) {
    int8_t result = -1;
 
    const uint8_t pageCount = 8;

    //int q = (float) i * 0.69 + 30;
    //Automation_Garage_StoredSettings_Typedef *pTemp = (Automation_Garage_StoredSettings_Typedef*) malloc(size);
    Automation_Garage_StoredSettings_Typedef pTemp = { 0 };
    HAL_EEPROM_Read(&heeprom, 0, (uint32_t*)&pTemp, pageCount, 100000);

     uint16_t indexCRC      = size - 2;
     uint16_t calculatedCRC = calcCRC((uint8_t*)&pTemp, indexCRC);
     uint16_t loadedCRC     = pTemp.crc;

     if (loadedCRC != calculatedCRC) {
        pdefaultSettings->crc = calcCRC((uint8_t*)&pTemp, indexCRC);
        HAL_EEPROM_Erase(&heeprom, 0, 32, HAL_EEPROM_WRITE_ALL, 100000);
        HAL_EEPROM_Write(&heeprom, 0, (uint32_t*)pdefaultSettings, 8, HAL_EEPROM_WRITE_ALL, 100000);
     } else {
        memcpy((void*)pdefaultSettings, (void*)&pTemp, size);
     }


    return result;
}

// void Automation_Garage_TestProceed1() {
//     int parrot = 0;
//     static int coef[] = { 1, 1, 1, 1 };
//     static uint16_t fillFactor[] = { 301, 301, 301, 301 };
//     static bool isFillDone[4] = { false, };
//     const uint16_t minChannelsBrightness = 300;
//     uint16_t currentIndex = 0;
//     bool isTestCommand = false;
    
//     //Modbus_Regmap_GetCopyOfItem(TEST_LIGHT, &isTestCommand, sizeof(bool));
//     isTestCommand = Modbus_Regmap_Coils[TEST_LIGHT];
//      if (isTestCommand) {
//         for (currentIndex = 0; currentIndex < 4; ++currentIndex) {

//             if (!isFillDone[currentIndex]) {
//                 // if (fillFactor[currentIndex] >= ((float)Modbus_Regmap_Settings[currentIndex] * 0.69 + 30) * 10 - 20) {
//                 //     coef[currentIndex] = -1;     
//                 // }
//                 coef[currentIndex] = fillFactor[currentIndex] >= 
//                                     ((float)Modbus_Regmap_Settings[currentIndex] * 0.69 + 30) * 10 - 20 ? - 1 : coef[currentIndex];  
//                 if (fillFactor[currentIndex] <= minChannelsBrightness) {
//                     coef[currentIndex] = 1;
//                     isFillDone[currentIndex] = true;
//                     if (isFillDone[0] && isFillDone[1] && isFillDone[2] && isFillDone[3]) {
//                         isTestCommand = false;
//                         //Modbus_Regmap_SetItem(TEST_LIGHT, &isTestCommand, sizeof(bool));
//                         Modbus_Regmap_Coils[TEST_LIGHT] = isTestCommand;
//                         for (uint8_t i = 0; i < 4; ++i) {
//                             isFillDone[i] = false;
//                         }
//                     }
//                 }
//                     // parrot = htimer32_1.Top * userPow(fillFactor[currentIndex], 2) / 5000;
//                     parrot = gammaCorrection[fillFactor[currentIndex]];
//                     HAL_Timer32_Channel_OCR_Set(timChannelMas[currentIndex], (parrot - 1) >> 1);
//                     fillFactor[currentIndex] += coef[currentIndex];
//             }   
//         }   
//         // currentIndex++;
//         // currentIndex = currentIndex >= 4 ? 0 : currentIndex; 
//     }  
// }

void Automation_Garage_TestProceed() {
    static int parrot = 0;
    static float coef[] = { 1, 1, 1, 1};
    static float fillFactor[] = { 301.0f, 301.0f, 301.0f, 301.0f,};
    static bool isFillDone[4] = { false, };
    static const float minChannelsBrightness = 300.0f;
    static uint16_t currentIndex = 0;
    static bool isTestCommand = false;
    static int8_t direction[] = { 1, 1, 1, 1 };
   
    isTestCommand = Modbus_Regmap_Coils[TEST_LIGHT];
     if (isTestCommand) {
       
        static float maxChannelsBrightness[4] = { 0 };
        static float physicalPercent = 0.0f;
        static const float tickTime = 0.0001f;
        const uint16_t updateTicks = Modbus_Regmap_Settings[TEST_LIGHT_B_TIME_ADDR % FOG_LIGHT_B_MAX_ADDR];
        float transitionTime =  8 * Modbus_Regmap_Settings[TEST_LIGHT_B_TIME_ADDR % FOG_LIGHT_B_MAX_ADDR];
        if (transitionTime < 0.1f) {
            transitionTime = 0.1f;
        }
        float updatePeriod = tickTime * updateTicks;
        if (updatePeriod < 0.1f) {
            updatePeriod = 0.1f;
        }
        float totalSteps = transitionTime / updatePeriod;
        const size_t gammaCorrectionSize = sizeof(gammaCorrection) / sizeof(gammaCorrection[0]) - 1;
        
        for (currentIndex = 0; currentIndex < 4; ++currentIndex) {
            if (!isFillDone[currentIndex]) {
                physicalPercent = (float)Modbus_Regmap_Settings[currentIndex] * 0.69f + 30.0f;
                maxChannelsBrightness[currentIndex] = physicalPercent * gammaCorrectionSize / 100.0f;
                float range = maxChannelsBrightness[currentIndex] - minChannelsBrightness;
                coef[currentIndex] = range / totalSteps;
                if (fillFactor[currentIndex] >= maxChannelsBrightness[currentIndex] - 20.0f &&  direction[currentIndex] > 0 ) {
                    direction[currentIndex] = -1;     
                }
                if (fillFactor[currentIndex] <= minChannelsBrightness + coef[currentIndex] && direction[currentIndex] < 0) {
                    direction[currentIndex] = 1; 
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
                uint16_t gammaIndex = (uint16_t)(fillFactor[currentIndex] + 0.5f);
                if (gammaIndex >= gammaCorrectionSize) {
                    gammaIndex = gammaCorrectionSize - 1;
                } else if (gammaIndex < minChannelsBrightness) {
                    gammaIndex = minChannelsBrightness;
                }
                parrot = gammaCorrection[gammaIndex];
                HAL_Timer32_Channel_OCR_Set(timChannelMas[currentIndex], (parrot - 1) >> 1);
                fillFactor[currentIndex] += coef[currentIndex] * direction[currentIndex];
                if (fillFactor[currentIndex] < minChannelsBrightness) fillFactor[currentIndex] = minChannelsBrightness;
                if (fillFactor[currentIndex] > maxChannelsBrightness[currentIndex]) fillFactor[currentIndex] = maxChannelsBrightness[currentIndex];
            }   
        }   
    }  
}


void Automation_Garage_SetBrightness() {
    static uint16_t oldValue = 0;
    //const uint8_t offset = 7U;     ///< Смещение старта нужных адресов фонарей в массиве
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
    currentIndex = currentIndex >= 5 ? 0 : currentIndex; 
}

void Automation_Garage_SetStopLight() {
    static uint32_t timeCounter = 0;
    static bool defaultValue = false;
    static bool strobeValue = false;
    static uint32_t defaultTimeValue = 5;
    static uint16_t timeValue = 0;
    static uint32_t parrot = 0;
    static uint16_t stopLightMaxBrightness = 0;
    
    static uint32_t tickSave = 0;
    static bool isTimeSaved = false;
    
    if (!Modbus_Regmap_Coils[TEST_LIGHT]) {
        timeValue = Modbus_Regmap_Settings[STOP_LIGHT_STROBE_TIME_ADDR % FOG_LIGHT_B_MAX_ADDR];
    if (timeValue != defaultTimeValue) {
        defaultTimeValue = timeValue;
    }
    defaultValue = Modbus_Regmap_Coils[STOP_LIGHT];
    strobeValue  = Modbus_Regmap_Coils[STOP_LIGHT_STROBE];
    if (defaultValue && !strobeValue) {
        isTimeSaved = false; 
        stopLightMaxBrightness  = Modbus_Regmap_Settings[STOP_LIGHT_B_MAX_ADDR % FOG_LIGHT_B_MAX_ADDR] * 0.69f + 30.0f;
        parrot = htimer32_1.Top * userPow(stopLightMaxBrightness, 2) / 5000;
        HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, (parrot - 1) >> 1);
    } else if (strobeValue) {
        if (!isTimeSaved) {
            tickSave = timeCounter;
            isTimeSaved = true;
        }
        if (timeCounter - tickSave >= (timeValue * 2)) {
            //Modbus_Regmap_SetItem(STOP_LIGHT_STROBE, &strobeValue, sizeof(bool));  
            stopLightMaxBrightness  = Modbus_Regmap_Settings[STOP_LIGHT_B_MAX_ADDR % FOG_LIGHT_B_MAX_ADDR];
            parrot = htimer32_1.Top * userPow(stopLightMaxBrightness, 2) / 5000;
            HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, (parrot - 1) >> 1);
        } else {
                if (!(timeCounter & 0x01)) {
                    stopLightMaxBrightness  = Modbus_Regmap_Settings[STOP_LIGHT_B_MAX_ADDR % FOG_LIGHT_B_MAX_ADDR];
                    parrot = htimer32_1.Top * userPow(stopLightMaxBrightness, 2) / 5000;
                HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, (parrot - 1) >> 1);
        } else {
                HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, 0 >> 1);
            }  
        } 
    } else {
            HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, 0 >> 1);
            isTimeSaved = false; 
        }
    }
    timeCounter++;
}

void setTurnPointer() {
    float turnLightMaxBrightness = 0;
    float coef = 1.0f;
    float maxChannelsBrightness = 0.0f;
    const float tickTime = 0.0001f;
    static float fillFactor = 301.0f;
    float transitionTime =  Modbus_Regmap_Settings[TEST_LIGHT_B_TIME_ADDR % FOG_LIGHT_B_MAX_ADDR];
    const uint16_t updateTicks =  8 * Modbus_Regmap_Settings[TEST_LIGHT_B_TIME_ADDR % FOG_LIGHT_B_MAX_ADDR];
    uint16_t turnModeValue = 0;
    float updatePeriod = tickTime * updateTicks;
    float totalSteps = transitionTime / updatePeriod;

    uint32_t parrot = 0;
    static uint32_t secCounter = 0;  ///< 10 тиков - 1 секунда
    static uint32_t timeSave = 0;
    const size_t gammaCorrectionSize = sizeof(gammaCorrection) / sizeof(gammaCorrection[0]) - 1;

    bool turnDefaultValue = false;
    static bool isRevertState = false;
    static bool isFillDone = false;

    if (!Modbus_Regmap_Coils[TEST_LIGHT]) {
        turnDefaultValue       = Modbus_Regmap_Coils[TURN_LIGHT];
        turnModeValue          = Modbus_Regmap_Settings[TURNING_LIGHT_MODE_ADDR % FOG_LIGHT_B_MAX_ADDR];
        turnLightMaxBrightness = (float)Modbus_Regmap_Settings[TURN_LIGHT_B_MAX_ADDR % FOG_LIGHT_B_MAX_ADDR] * 0.69 + 30.0f;

        if (turnDefaultValue && !turnModeValue) {
            if (!(secCounter % 50)) {
                parrot = htimer32_1.Top * userPow(turnLightMaxBrightness, 2) / 5000;
                if (!isRevertState) {
                    HAL_Timer32_Channel_OCR_Set(&htimer32_channel0, (parrot - 1) >> 1);
                    isRevertState = true;
                } else {
                    HAL_Timer32_Channel_OCR_Set(&htimer32_channel0, 0 >> 1);
                    isRevertState = false;
                }        
            }
        } else if (turnDefaultValue && turnModeValue) {
            if (secCounter - timeSave >= 50 && isFillDone) {
                isFillDone = false;  
            } else if (!isFillDone) {
                maxChannelsBrightness = turnLightMaxBrightness * gammaCorrectionSize / 100.0f;
                float range = maxChannelsBrightness - 300.0f;
                coef = range / totalSteps;
                if (fillFactor >= maxChannelsBrightness - 20.0f) {
                    fillFactor = 302.0f;
                    isFillDone = true;
                    timeSave = secCounter;
                    HAL_Timer32_Channel_OCR_Set(&htimer32_channel0, 0 >> 1);
                }
                //parrot = htimer32_1.Top * userPow(fillFactor, 2) / 5000;
                uint32_t gammaIndex = (uint32_t)(fillFactor + 0.5f);
                if (gammaIndex >= gammaCorrectionSize) {
                    gammaIndex = gammaCorrectionSize - 1;
                }
                parrot = gammaCorrection[gammaIndex];
                HAL_Timer32_Channel_OCR_Set(&htimer32_channel0, (parrot - 1) >> 1);
                fillFactor += coef;
            }
        } else {
            HAL_Timer32_Channel_OCR_Set(&htimer32_channel0, 0 >> 1);
            fillFactor = 301.0f;
        }
        secCounter++;
    }
    
}

// void setTurnPointer() {
//     static bool turnDefaultValue = false;
//     static uint16_t turnModeValue = 0;
//     static uint32_t parrot = 0;
//     static uint16_t turnLightMaxBrightness = 0;
//     static uint32_t secCounter = 0;  ///< 10 тиков - 1 секунда
//     static bool isRevertState = false;
//     static uint32_t fillFactor = 301;
//     static bool isFillDone = false;
//     static uint32_t timeSave = 0;

//     //Modbus_Regmap_GetCopyOfItem(TURN_LIGHT, &turnDefaultValue, sizeof(bool));
//     // Modbus_Regmap_GetCopyOfItem(TURNING_LIGHT_MODE_ADDR, &turnModeValue, sizeof(uint16_t));
//     // Modbus_Regmap_GetCopyOfItem(TURN_LIGHT_B_MAX_ADDR, &turnLightMaxBrightness, sizeof(uint16_t));
//     if (!Modbus_Regmap_Coils[TEST_LIGHT]) {
//         turnDefaultValue       = Modbus_Regmap_Coils[TURN_LIGHT];
//         turnModeValue          = Modbus_Regmap_Settings[TURNING_LIGHT_MODE_ADDR % FOG_LIGHT_B_MAX_ADDR];
//         turnLightMaxBrightness = Modbus_Regmap_Settings[TURN_LIGHT_B_MAX_ADDR % FOG_LIGHT_B_MAX_ADDR];

//         if (turnDefaultValue && !turnModeValue) {
//             if (!(secCounter % 50)) {
//                 parrot = htimer32_1.Top * userPow(turnLightMaxBrightness, 2) / 5000;
//                 if (!isRevertState) {
//                     HAL_Timer32_Channel_OCR_Set(&htimer32_channel0, (parrot - 1) >> 1);
//                     isRevertState = true;
//                 } else {
//                     HAL_Timer32_Channel_OCR_Set(&htimer32_channel0, 0 >> 1);
//                     isRevertState = false;
//                 }        
//             }
//         } else if (turnDefaultValue && turnModeValue) {
//             if (secCounter - timeSave >= 50 && isFillDone) {
//                 isFillDone = false;  
//             } else if (!isFillDone) {
//                 if (fillFactor >= (turnLightMaxBrightness * 0.69 + 30) * 10 - 20) {
//                     fillFactor = 302;
//                     isFillDone = true;
//                     timeSave = secCounter;
//                     HAL_Timer32_Channel_OCR_Set(&htimer32_channel0, 0 >> 1);
//                 }
//                 //parrot = htimer32_1.Top * userPow(fillFactor, 2) / 5000;
//                 parrot = gammaCorrection[fillFactor];
//                 HAL_Timer32_Channel_OCR_Set(&htimer32_channel0, (parrot - 1) >> 1);
//                 fillFactor++;
//             }
//         } else {
//             HAL_Timer32_Channel_OCR_Set(&htimer32_channel0, 0 >> 1);
//             fillFactor = 2;
//         }
//         secCounter++;
//     }
    
// }

void setReverseLight() {
    uint32_t reverseLightMaxBrightness = 0;
    uint32_t parrot = 0;
    bool isReverseLight = false;

    reverseLightMaxBrightness = Modbus_Regmap_Settings[REVERSING_LIGHT_B_MAX_ADDR % FOG_LIGHT_B_MAX_ADDR] * 0.69f + 30.0f;
    if (!Modbus_Regmap_Coils[TEST_LIGHT]) {
        isReverseLight = Modbus_Regmap_Coils[REVERSING_LIGHT];
        if (isReverseLight) {
            parrot = htimer32_1.Top * userPow(reverseLightMaxBrightness, 2) / 5000;
            HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, (parrot - 1) >> 1);
        } else {
            HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, 0 >> 1);
        }
    } 
}


/// ПАМЯТКА КАК ДЕЛАТЬ НЕ НАДО! ЕЩЁ НИЖЕ ПЕРЕПИСАННАЯ ФУНКЦИЯ ///

// void setFogLight1() {
//     bool isFogLight = false;
//     bool isParkingLight = false;
//     uint16_t fogLightMaxBrightness = 0;
//     uint16_t parkingLightMaxBrightness = 0;
//     uint32_t parrot = 0;
//     static uint8_t fillFactor = 1;
//     static int coef = 1;

//     if (!Modbus_Regmap_Coils[TEST_LIGHT]) {
//         isFogLight                = Modbus_Regmap_Coils[FOG_LIGHT_ADDR];
//         isParkingLight            = Modbus_Regmap_Coils[PARKING_LIGHT];
//         fogLightMaxBrightness     = Modbus_Regmap_Settings[FOG_LIGHT_B_MAX_ADDR % FOG_LIGHT_B_MAX_ADDR];
//         parkingLightMaxBrightness = Modbus_Regmap_Settings[PARKING_LIGHT_B_MAX_ADDR % FOG_LIGHT_B_MAX_ADDR];
//         if (!isFogLight && isParkingLight) {
            
//             parkingLightMaxBrightness = parkingLightMaxBrightness > 97 ? 97 : parkingLightMaxBrightness;
//             if (fillFactor >= parkingLightMaxBrightness) {
//                 coef = 0;
//                 fillFactor = 1;
//             } 
//             //parrot = htimer32_1.Top * userPow(fillFactor, 2) / 5000;
           
//             parrot = gammaCorrection[fillFactor];
          
//             HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, (parrot - 1) >> 1);
            
//             fillFactor += coef;
//         } else if (isFogLight) {
//             coef = 0;
//             parrot = htimer32_1.Top * userPow(fogLightMaxBrightness, 2) / 5000;
//             HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, (parrot - 1) >> 1);    
//         } else {
//             coef = -1;
//             if (fillFactor <= 1) {
//                 coef = 0;
//                 HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, 0 >> 1);
//             } else {
//                 parrot = gammaCorrection[fillFactor];
//                 HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, (parrot - 1) >> 1);
//                 fillFactor += coef;
//             }     
//         }
//     }
// }

void setFogLight() {
    bool isFogLight = false;
    bool isParkingLight = false;
    uint16_t fogLightMaxBrightness = 0;
    uint16_t parkingLightMaxBrightness = 0;
    uint32_t parrot = 0;
    static bool isFilledOnce = false;

    float turnLightMaxBrightness = 0;
    float coef = 1.0f;
    float maxChannelsBrightness = 0.0f;
    const float tickTime = 0.0001f;
    static float fillFactor = 301.0f;
    float transitionTime =  Modbus_Regmap_Settings[TEST_LIGHT_B_TIME_ADDR % FOG_LIGHT_B_MAX_ADDR];
    const uint16_t updateTicks =  10 * Modbus_Regmap_Settings[TEST_LIGHT_B_TIME_ADDR % FOG_LIGHT_B_MAX_ADDR];
    uint16_t turnModeValue = 0;
    float updatePeriod = tickTime * updateTicks;
    float totalSteps = transitionTime / updatePeriod;
    const size_t gammaCorrectionSize = sizeof(gammaCorrection) / sizeof(gammaCorrection[0]) - 1;

    if (!Modbus_Regmap_Coils[TEST_LIGHT]) {
        isFogLight                = Modbus_Regmap_Coils[FOG_LIGHT_ADDR];
        isParkingLight            = Modbus_Regmap_Coils[PARKING_LIGHT];
        fogLightMaxBrightness     = Modbus_Regmap_Settings[FOG_LIGHT_B_MAX_ADDR % FOG_LIGHT_B_MAX_ADDR];
        turnLightMaxBrightness    = Modbus_Regmap_Settings[PARKING_LIGHT_B_MAX_ADDR % FOG_LIGHT_B_MAX_ADDR] * 0.69 + 30;

        if (!isFogLight && !isParkingLight) {
            isFilledOnce = false;
            if (fillFactor > 302) {
                parrot = gammaCorrection[(int)fillFactor];
                HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, (parrot - 1) >> 1);
                fillFactor -= coef;
            } else {
                HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, 0 >> 1);
            }
        } else if (!isFogLight && isParkingLight) {
            if (fillFactor <= (turnLightMaxBrightness * 10 - 20) && !isFilledOnce) {
                maxChannelsBrightness = turnLightMaxBrightness * gammaCorrectionSize / 100.0f;
                float range = maxChannelsBrightness - 300.0f;
                coef = range / totalSteps;

                parrot = gammaCorrection[(int)fillFactor];
                HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, (parrot - 1) >> 1);
                fillFactor += coef;
            } else {
                isFilledOnce = true;
                fillFactor = turnLightMaxBrightness * 10 - 20;
                parrot = gammaCorrection[(int)fillFactor];
                HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, (parrot - 1) >> 1);
            }
        } else if (isFogLight) {
            parrot = htimer32_1.Top * userPow(fogLightMaxBrightness, 2) / 5000;
            HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, (parrot - 1) >> 1); 
        }
    }
}
