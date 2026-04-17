
#include "mik32_hal_spifi_w25.h"
#include "mik32_hal_dma.h"
#include "Modbus.h"
#include "Modbus_Config.h"
#include "Automation_Garage.h"

/**
 * @file main.c
 * @brief Пример портирования ОСРВ FreeRTOS для MIK32
 * 
 * Подразумевается сборка примера со скриптом линковки spifi.ld и загрузка 
 * прошивки в Flash память.
 * 
 * В примере показано создание задач и семафоров на примере мигания светодиодом
 * и обработки нажатия кнопки. 
 * 
 * В процессе инициализации настраивается четырехканальный режим Flash памяти и
 * включается кэш интерфейса SPIFI. Но, если требуется получение максимальной 
 * производительности, потребуется включить режимы QPI и Continuous Read 
 * с помощью примера HAL_SPI_Flash_Init, записанного во внутренний EEPROM, или 
 * загрузчика.
 * 
 * Для управления размером области кучи, управляемой FreeRTOS, используется
 * параметр configTOTAL_HEAP_SIZE в файле FreeRTOSConfig.h
 * 
 */

#define LED1_PIN GPIO_PIN_3
#define LED1_PORT GPIO_1

#define LED2_PIN GPIO_PIN_3
#define LED2_PORT GPIO_0

#define BTN_PIN GPIO_PIN_8
#define BTN_PORT GPIO_0
#define BTN_IRQ_LINE GPIO_LINE_0
#define BTN_IRQ_LINE_MUX GPIO_MUX_PORT0_8_LINE_0

#define _PWM_PORT GPIO_0 
#define _PWM_PIN_0 GPIO_PIN_0
#define _PWM_PIN_1 GPIO_PIN_1
#define _PWM_PIN_2 GPIO_PIN_2
#define _PWM_PIN_3 GPIO_PIN_3

#define BUFFER_LENGTH 50

TIMER32_HandleTypeDef htimer32_1;
TIMER32_CHANNEL_HandleTypeDef htimer32_channel0;
TIMER32_CHANNEL_HandleTypeDef htimer32_channel1;
TIMER32_CHANNEL_HandleTypeDef htimer32_channel2;
TIMER32_CHANNEL_HandleTypeDef htimer32_channel3;

TIMER32_HandleTypeDef htimer32_2;

USART_HandleTypeDef husart1;
uint8_t bufPointer;
const uint8_t Modbus_FrameCount = 8;
uint32_t tickCounter;

DMA_InitTypeDef hdma;
DMA_ChannelHandleTypeDef hdma_ch1;

WDT_HandleTypeDef hwdt;

HAL_EEPROM_HandleTypeDef heeprom;

extern modbusHandler_t mHandle;
__attribute__((section(".ram_text"))) void Startup_SPIFI_Config();
// Структура для параметров задачи blink_task.
typedef struct
{
	GPIO_TypeDef *port;	 
	HAL_PinsTypeDef pin; 
	uint32_t interval;	 
} led_config_t;

void SystemClock_Config();
void GPIO_Init();
void SPIFI_Init();
void USART_Init();
void EEPROM_Init();
static void Timer32_1_Init();
static void Timer32_2_Init();
void initFunc();

typedef struct {
    GPIO_TypeDef *GPIOs;
    HAL_PinsTypeDef Pins; 
}GPIOs_TypeDef;

GPIOs_TypeDef flashlights[] = {
    {_PWM_PORT, _PWM_PIN_0},
	{_PWM_PORT, _PWM_PIN_1},
	{_PWM_PORT, _PWM_PIN_2},
	{_PWM_PORT, _PWM_PIN_3},
};

extern ModbusSettings_TypeDef MbSettings;
void initFunc()
{
	Auromation_Garage_InitDefaultSettings();
     
    HAL_EPIC_MaskLevelSet(HAL_EPIC_UART_1_MASK | HAL_EPIC_TIMER32_2_MASK); 
    HAL_IRQ_EnableInterrupts();
    HAL_USART_RXNE_EnableInterrupt(&husart1);
    HAL_USART_IDLE_EnableInterrupt(&husart1);
    HAL_USART_RX_Error_EnableInterrupt(&husart1);
    
    HAL_Timer32_Channel_Enable(&htimer32_channel0);
	HAL_Timer32_Channel_Enable(&htimer32_channel1);
	HAL_Timer32_Channel_Enable(&htimer32_channel2);
    HAL_Timer32_Channel_Enable(&htimer32_channel3);

    HAL_Timer32_Value_Clear(&htimer32_1);
    HAL_Timer32_Start(&htimer32_1);

    HAL_Timer32_Channel_OCR_Set(&htimer32_channel0, 0 >> 1);
    HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, 0 >> 1);
    HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, 0 >> 1);
    HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, 0 >> 1);

    HAL_Timer32_Value_Clear(&htimer32_2);
    HAL_Timer32_Base_Start_IT(&htimer32_2);
    

	ModbusInit(&mHandle);

    // for (int i = 0; i < 6; ++i) {
    //             HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_6);
    //             HAL_DelayMs(150);
    //         }
}
extern uint16_t Modbus_Regmap_Settings[NUMBER_OF_SETTINGS];
typedef struct {
    uint32_t period;
    uint32_t last;
    void(*handler)();
}Task_TypeDef;

uint32_t getTicks() {
    return tickCounter;
}

void togglePin() {
    HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_6);
}



int main()
{
	HAL_Init();

	SystemClock_Config();
    // WDT_Init();
    // HAL_WDT_Start(&hwdt, WDT_TIMEOUT_DEFAULT);
    
    
	Timer32_1_Init();
    Timer32_2_Init();
	GPIO_Init();
    
	USART_Init();
    EEPROM_Init();
	SPIFI_Init(); 					///< Для работы с QPSIFI
	//Startup_SPIFI_Config();		///< Для работы с SPIFI

	//HAL_GPIO_ClearInterrupts();
  
    initFunc();
	HAL_EPIC_Clear(0xFFFFFFFF);
	
Task_TypeDef tasks[] = {
    { 0, 0, Automation_Garage_TestProceed }, 
    { 300, 0, Automation_Garage_SetBrightness },
    { 500, 0, togglePin },
    { 500, 0, Automation_Garage_SetStopLight },
    { 20, 0, setTurnPointer },
    { 100, 0, setReverseLight },
    { 0, 0, setFogLight },
    { 150, 0, Automation_Garage_SaveSettings},
    { 80, 0, Automation_Garage_FlashErase },
};


    while (1) {
       //HAL_WDT_Refresh(&hwdt, WDT_TIMEOUT_DEFAULT);
        for (int i = 0; i < 9; ++i) {
            tasks[i].period = i == 0 ? 2 * Modbus_Regmap_Settings[TEST_LIGHT_B_TIME_ADDR % FOG_LIGHT_B_MAX_ADDR] : tasks[i].period;
            tasks[i].period = i == 6 ? 25 * Modbus_Regmap_Settings[PARKING_LIGHT_B_TIME_ADDR % FOG_LIGHT_B_MAX_ADDR] : tasks[i].period;
            if (getTicks() - tasks[i].last >= tasks[i].period) {
                tasks[i].last += tasks[i].period;
                tasks[i].handler();
            }  
        } 
    }
}

void SystemClock_Config(void)
{
	PCC_InitTypeDef PCC_OscInit = {0};

	PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_OSC32M;
	PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
	PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
	PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_LSI32K;
	PCC_OscInit.AHBDivider = 0;
	PCC_OscInit.APBMDivider = 0;
	PCC_OscInit.APBPDivider = 0;
	// PCC_OscInit.HSI32MCalibrationValue = 127;
	PCC_OscInit.LSI32KCalibrationValue = 8;
	PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_LSI32K;
	PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_LSI32K;
    HAL_PCC_Config(&PCC_OscInit);
}

void GPIO_Init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_PCC_GPIO_0_CLK_ENABLE();
	__HAL_PCC_GPIO_1_CLK_ENABLE();
	__HAL_PCC_GPIO_2_CLK_ENABLE();
    //__HAL_PCC_WDT_CLK_ENABLE();
	//__HAL_PCC_GPIO_IRQ_CLK_ENABLE();

	// Инициализация LED1 и LED2
	GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
	GPIO_InitStruct.Pull = HAL_GPIO_PULL_NONE;
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	HAL_GPIO_Init(GPIO_2, &GPIO_InitStruct);

	
	GPIO_InitStruct.Pin = USART1_REDE_PIN;
	HAL_GPIO_Init(USART1_REDE_PORT, &GPIO_InitStruct);

}


SPIFI_HandleTypeDef spifi = {
        .Instance = SPIFI_CONFIG, 
        .cacheEnabled = SPIFI_CACHE_ENABLE, 
        .dataCacheEnabled = SPIFI_DATA_CACHE_ENABLE,
    };
    
void SPIFI_Init()
{
    HAL_SPIFI_W25_InitQuadModeFromFlash(&spifi);
}

uint8_t FLASH_ERASE = 0;
void EEPROM_Init()
{
    heeprom.Instance = EEPROM_REGS;
    heeprom.Mode = HAL_EEPROM_MODE_TWO_STAGE;
    heeprom.ErrorCorrection = HAL_EEPROM_ECC_ENABLE;
    heeprom.EnableInterrupt = HAL_EEPROM_SERR_DISABLE;

    HAL_EEPROM_Init(&heeprom);
    HAL_EEPROM_CalculateTimings(&heeprom, OSC_SYSTEM_VALUE);

    if (FLASH_ERASE) {
        const uint8_t pageWords = 32;
        HAL_EEPROM_Erase(&heeprom, 0, pageWords, HAL_EEPROM_WRITE_ALL, 100000);
    }
}

__attribute__((section(".ram_text"))) void Startup_SPIFI_Config()
{
    HAL_SPIFI_MspInit_LL();

    SPIFI_HandleTypeDef spifi = {.Instance = SPIFI_CONFIG};

    const uint32_t CMD_READ_DATA =
        SPIFI_DIRECTION_INPUT | SPIFI_CONFIG_CMD_INTLEN(0) |
        SPIFI_CONFIG_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
        SPIFI_CONFIG_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OPCODE_3ADDR) |
        SPIFI_CONFIG_CMD_OPCODE(0x03);
	
    HAL_SPIFI_MemoryMode_Init_LL(&spifi,
                                 0x400FFFFF | SPIFI_CONFIG_CTRL_CACHE_EN_M,
                                 0x90000000, CMD_READ_DATA);
							 
}


void trap_handler()
{
     if (EPIC_CHECK_UART_1()) {
		// --- Проверка ORE и сброс ---
        if (husart1.Instance->FLAGS & UART_FLAGS_ORE_M) {
            //HAL_USART_ReceiveOverwrite_ClearFlag(&husart1);
            UART_1->FLAGS |= UART_FLAGS_ORE_M;
        }

        // --- Чтение всех пришедших байт ---
        if (UART_1->FLAGS & UART_FLAGS_RXNE_M) { 
            uint8_t byte = 0;
            byte = (uint8_t)UART_1->RXDATA;
            if (bufPointer < BUFFER_LENGTH) {
                mHandlers[0]->u8Buffer[bufPointer++] = byte;
            }
			//HAL_USART_RXNE_ClearFlag(&husart1);
            UART_1->FLAGS |= UART_FLAGS_RXNE_M;
        }
   
        // --- Проверка IDLE (конец кадра) ---
        if (UART_1->FLAGS & UART_FLAGS_IDLE_M) {
            StartTaskModbusSlave(mHandlers[0], bufPointer);
			
            bufPointer = 0;   
            UART_1->FLAGS |= UART_FLAGS_IDLE_M;
        }
    }
    if (EPIC_CHECK_TIMER32_2()) {
        tickCounter++;
        HAL_TIMER32_INTERRUPTFLAGS_CLEAR(&htimer32_2);
    }  
    HAL_EPIC_Clear(EPIC_LINE_UART_1_S | EPIC_LINE_TIMER32_2_S);
}


static void Timer32_1_Init() {
    htimer32_1.Instance = TIMER32_1;
    htimer32_1.Top = 6400 - 1;
    htimer32_1.State = TIMER32_STATE_DISABLE;
    htimer32_1.Clock.Source = TIMER32_SOURCE_PRESCALER;
    htimer32_1.Clock.Prescaler = 0;
    htimer32_1.InterruptMask = 0;
    htimer32_1.CountMode = TIMER32_COUNTMODE_FORWARD;
    HAL_Timer32_Init(&htimer32_1);

    htimer32_channel0.TimerInstance = htimer32_1.Instance;
    htimer32_channel0.ChannelIndex = TIMER32_CHANNEL_0;
    htimer32_channel0.PWM_Invert = TIMER32_CHANNEL_INVERTED_PWM;
    htimer32_channel0.Mode = TIMER32_CHANNEL_MODE_PWM;
    htimer32_channel0.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    htimer32_channel0.OCR = (6400 - 1) >> 1;
    htimer32_channel0.Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(&htimer32_channel0);

	htimer32_channel1.TimerInstance = htimer32_1.Instance;
    htimer32_channel1.ChannelIndex = TIMER32_CHANNEL_1;
    htimer32_channel1.PWM_Invert = TIMER32_CHANNEL_INVERTED_PWM;
    htimer32_channel1.Mode = TIMER32_CHANNEL_MODE_PWM;
    htimer32_channel1.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    htimer32_channel1.OCR = (6400 - 1) >> 1;
    htimer32_channel1.Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(&htimer32_channel1);

	htimer32_channel2.TimerInstance = htimer32_1.Instance;
    htimer32_channel2.ChannelIndex = TIMER32_CHANNEL_2;
    htimer32_channel2.PWM_Invert = TIMER32_CHANNEL_INVERTED_PWM;
    htimer32_channel2.Mode = TIMER32_CHANNEL_MODE_PWM;
    htimer32_channel2.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    htimer32_channel2.OCR = (6400 - 1) >> 1;
    htimer32_channel2.Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(&htimer32_channel2);

    htimer32_channel3.TimerInstance = htimer32_1.Instance;
    htimer32_channel3.ChannelIndex = TIMER32_CHANNEL_3;
    htimer32_channel3.PWM_Invert = TIMER32_CHANNEL_INVERTED_PWM;
    htimer32_channel3.Mode = TIMER32_CHANNEL_MODE_PWM;
    htimer32_channel3.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    htimer32_channel3.OCR = (6400 - 1) >> 1;
    htimer32_channel3.Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(&htimer32_channel3);
}

static void Timer32_2_Init() {
    htimer32_2.Instance = TIMER32_2;
    htimer32_2.Top = 32 - 1;
    htimer32_2.State = TIMER32_STATE_DISABLE;
    htimer32_2.Clock.Source = TIMER32_SOURCE_PRESCALER;
    htimer32_2.Clock.Prescaler = 1000 - 1;
    htimer32_2.InterruptMask = 0;
    htimer32_2.CountMode = TIMER32_COUNTMODE_FORWARD;
    HAL_Timer32_Init(&htimer32_2);
}

void USART_Init()
{
    husart1.Instance = UART_1;
    husart1.transmitting = Enable;
    husart1.receiving = Enable;
    husart1.frame = Frame_8bit;
    husart1.parity_bit = Disable;
    husart1.parity_bit_inversion = Disable;
    husart1.bit_direction = LSB_First;
    husart1.data_inversion = Disable;
    husart1.tx_inversion = Disable;
    husart1.rx_inversion = Disable;
    husart1.swap = Disable;
    husart1.lbm = Disable;
    husart1.stop_bit = StopBit_1;
    husart1.mode = Asynchronous_Mode;
    husart1.xck_mode = XCK_Mode3;
    husart1.last_byte_clock = Disable;
    husart1.overwrite = Disable;
    husart1.rts_mode = AlwaysEnable_mode;
    husart1.dma_tx_request = Disable;
    husart1.dma_rx_request = Disable;
    husart1.channel_mode = Duplex_Mode;
    husart1.tx_break_mode = Disable;
    husart1.Interrupt.ctsie = Disable;
    husart1.Interrupt.eie = Disable;
    husart1.Interrupt.idleie = Disable;
    husart1.Interrupt.lbdie = Disable;
    husart1.Interrupt.peie = Disable;
    husart1.Interrupt.rxneie = Disable;
    husart1.Interrupt.tcie = Disable;
    husart1.Interrupt.txeie = Disable;
    husart1.Modem.rts = Disable; //out
    husart1.Modem.cts = Disable; //in
    husart1.Modem.dtr = Disable; //out
    husart1.Modem.dcd = Disable; //in
    husart1.Modem.dsr = Disable; //in
    husart1.Modem.ri = Disable;  //in
    husart1.Modem.ddis = Disable;//out
    husart1.baudrate = 115200;
    HAL_USART_Init(&husart1);
}