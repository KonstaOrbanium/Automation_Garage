
#include "mik32_hal_spifi_w25.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

#include "xprintf.h"

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


extern modbusHandler_t mHandle;
SemaphoreHandle_t xButtonSemaphore;
__attribute__((section(".ram_text"))) void Startup_SPIFI_Config();
// Структура для параметров задачи blink_task.
typedef struct
{
	GPIO_TypeDef *port;	 // Указатель на порт.
	HAL_PinsTypeDef pin; // Номер пина.
	uint32_t interval;	 // Период смены состояния в миллисекундах.
} led_config_t;

void SystemClock_Config();
void GPIO_Init();
void SPIFI_Init();
void USART_Init();
static void Timer32_1_Init();
static void Timer32_2_Init();

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

// Задача мигания светодиодом.
void blink_task(void *pvParameters)
{
	Automation_Garage_InitAllObjects();
     
    HAL_EPIC_MaskLevelSet(HAL_EPIC_UART_1_MASK | HAL_EPIC_TIMER32_2_MASK); 
    HAL_IRQ_EnableInterrupts();
    HAL_USART_RXNE_EnableInterrupt(&husart1);
    HAL_USART_IDLE_EnableInterrupt(&husart1);
    HAL_USART_RX_Error_EnableInterrupt(&husart1);
    
    // HAL_Timer32_Channel_Enable(&htimer32_channel0);
	// HAL_Timer32_Channel_Enable(&htimer32_channel1);
	// HAL_Timer32_Channel_Enable(&htimer32_channel2);
    // HAL_Timer32_Channel_Enable(&htimer32_channel3);

    // HAL_Timer32_Value_Clear(&htimer32_1);
    // HAL_Timer32_Start(&htimer32_1);

    // HAL_Timer32_Channel_OCR_Set(&htimer32_channel0, 0 >> 1);
    // HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, 0 >> 1);
    // HAL_Timer32_Channel_OCR_Set(&htimer32_channel2, 0 >> 1);
    // HAL_Timer32_Channel_OCR_Set(&htimer32_channel3, 0 >> 1);

    // HAL_Timer32_Value_Clear(&htimer32_2);
    // HAL_Timer32_Base_Start_IT(&htimer32_2);
    //HAL_USART_TXE_EnableInterrupt(&husart1);

	ModbusInit(&mHandle);

	// for (;;)
	// { 
	//     vTaskDelay(pdMS_TO_TICKS(40));
			
	// }
	vTaskDelete((TaskHandle_t)NULL);
    // for (int i = 0; i < 6; ++i) {
    //             HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_6);
    //             HAL_DelayMs(150);
    //         }
}


int main()
{
	HAL_Init();

	SystemClock_Config();
    HAL_DelayMs(300);
    
	Timer32_1_Init();
    Timer32_2_Init();
	GPIO_Init();
   
	USART_Init();

	SPIFI_Init(); 					///< Для работы с QPSIFI
	//Startup_SPIFI_Config();		///< Для работы с SPIFI

    // for (int i = 0; i < 6; ++i) {
    //             HAL_GPIO_TogglePin(GPIO_1, GPIO_PIN_3);
    //             HAL_DelayMs(150);
    //         }

	//HAL_GPIO_ClearInterrupts();
	HAL_EPIC_Clear(0xFFFFFFFF);
	
	// Создание бинарного семафора.
	xButtonSemaphore = xSemaphoreCreateBinary();
	
	// Создание задач.
	xTaskCreate(blink_task,
				"LED1",
				512,
				NULL,
				tskIDLE_PRIORITY + 1,
				NULL);

	vTaskStartScheduler();
}

void SystemClock_Config(void)
{
	PCC_InitTypeDef PCC_OscInit = {0};

	PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_OSC32M;
	PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
	PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
	PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
	PCC_OscInit.AHBDivider = 0;
	PCC_OscInit.APBMDivider = 0;
	PCC_OscInit.APBPDivider = 0;
	// PCC_OscInit.HSI32MCalibrationValue = 127;
	// PCC_OscInit.LSI32KCalibrationValue = 8;
	PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_AUTO;
	PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
	HAL_PCC_Config(&PCC_OscInit);
}

void GPIO_Init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_PCC_GPIO_0_CLK_ENABLE();
	__HAL_PCC_GPIO_1_CLK_ENABLE();
	__HAL_PCC_GPIO_2_CLK_ENABLE();
	//__HAL_PCC_GPIO_IRQ_CLK_ENABLE();

	// Инициализация LED1 и LED2
	GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
	GPIO_InitStruct.Pull = HAL_GPIO_PULL_NONE;
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	HAL_GPIO_Init(GPIO_2, &GPIO_InitStruct);

	
	GPIO_InitStruct.Pin = USART1_REDE_PIN;
	HAL_GPIO_Init(USART1_REDE_PORT, &GPIO_InitStruct);

    // GPIO_InitStruct.Pin = GPIO_PIN_0;
	// HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);

    // GPIO_InitStruct.Pin = GPIO_PIN_1;
	// HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);

    // GPIO_InitStruct.Pin = GPIO_PIN_2;
	// HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);

    // GPIO_InitStruct.Pin = GPIO_PIN_3;
	// HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);



	// // Инициализация пользовательской кнопки
	// GPIO_InitStruct.Pin = BTN_PIN;
	// GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_INPUT;
	// HAL_GPIO_Init(BTN_PORT, &GPIO_InitStruct);
	// HAL_GPIO_InitInterruptLine(BTN_IRQ_LINE_MUX, GPIO_INT_MODE_FALLING);
}


void SPIFI_Init()
{
    SPIFI_HandleTypeDef spifi = {
        .Instance = SPIFI_CONFIG, 
        .cacheEnabled = SPIFI_CACHE_ENABLE, 
        .dataCacheEnabled = SPIFI_DATA_CACHE_DISABLE,
    };
    HAL_SPIFI_W25_InitQuadModeFromFlash(&spifi);
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

extern void freertos_risc_v_trap_handler();
RAM_ATTR void raw_trap_handler()
{
	freertos_risc_v_trap_handler();
}



RAM_ATTR void freertos_risc_v_application_interrupt_handler(void) {
    if (EPIC_CHECK_UART_1()) {

		// --- Проверка ORE и сброс ---
        if (husart1.Instance->FLAGS & UART_FLAGS_ORE_M) {
            HAL_USART_ReceiveOverwrite_ClearFlag(&husart1);
        }

        // --- Чтение всех пришедших байт ---
        if (HAL_USART_RXNE_ReadFlag(&husart1)) { 
            uint8_t byte = 0;
            HAL_USART_Receive(&husart1, (char*)&byte, 0);
            if (bufPointer < BUFFER_LENGTH) {
                mHandlers[0]->u8RxBuffer[bufPointer++] = byte;
            }
			HAL_USART_RXNE_ClearFlag(&husart1);
        }
   
        // --- Проверка IDLE (конец кадра) ---
        if (HAL_USART_IDLE_ReadFlag(&husart1)) {
            HAL_USART_IDLE_ClearFlag(&husart1);

            //uint8_t buf[] = { 0x01, 0x01, 0x01, 0x00, 0x51, 0x88 };

            // HAL_GPIO_WritePin(USART1_REDE_PORT, USART1_REDE_PIN, GPIO_PIN_HIGH);
            // HAL_USART_Write(&husart1, (char*)buf, 8, 0);
            // HAL_USART_TXC_ClearFlag(&husart1);
            // HAL_GPIO_WritePin(USART1_REDE_PORT, USART1_REDE_PIN, GPIO_PIN_LOW);
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            uint8_t size = bufPointer;

            xQueueSendFromISR(
                mHandlers[0]->queueTaskSlaveHandle,
                &size,
                &xHigherPriorityTaskWoken
            );
			
            bufPointer = 0;
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    // static uint32_t tickCounter = 0;
    // if (EPIC_CHECK_TIMER32_2()) {
     
    //     if (!(tickCounter % 250)) 
    //         Automation_Garage_TestProceed();

    //     if (!(tickCounter % 10000)) {
    //         HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_6);
    //     }
    //     tickCounter++;
    //     HAL_TIMER32_INTERRUPTFLAGS_CLEAR(&htimer32_2);
    // }  
    HAL_EPIC_Clear(0xFFFFFFFF);
}


static void Timer32_1_Init() {
    htimer32_1.Instance = TIMER32_1;
    htimer32_1.Top = 32000 - 1;
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
    htimer32_channel0.OCR = (32000 - 1) >> 1;
    htimer32_channel0.Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(&htimer32_channel0);

	htimer32_channel1.TimerInstance = htimer32_1.Instance;
    htimer32_channel1.ChannelIndex = TIMER32_CHANNEL_1;
    htimer32_channel1.PWM_Invert = TIMER32_CHANNEL_INVERTED_PWM;
    htimer32_channel1.Mode = TIMER32_CHANNEL_MODE_PWM;
    htimer32_channel1.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    htimer32_channel1.OCR = (32000 - 1) >> 1;
    htimer32_channel1.Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(&htimer32_channel1);

	htimer32_channel2.TimerInstance = htimer32_1.Instance;
    htimer32_channel2.ChannelIndex = TIMER32_CHANNEL_2;
    htimer32_channel2.PWM_Invert = TIMER32_CHANNEL_INVERTED_PWM;
    htimer32_channel2.Mode = TIMER32_CHANNEL_MODE_PWM;
    htimer32_channel2.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    htimer32_channel2.OCR = (32000 - 1) >> 1;
    htimer32_channel2.Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(&htimer32_channel2);

    htimer32_channel3.TimerInstance = htimer32_1.Instance;
    htimer32_channel3.ChannelIndex = TIMER32_CHANNEL_3;
    htimer32_channel3.PWM_Invert = TIMER32_CHANNEL_INVERTED_PWM;
    htimer32_channel3.Mode = TIMER32_CHANNEL_MODE_PWM;
    htimer32_channel3.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    htimer32_channel3.OCR = (32000 - 1) >> 1;
    htimer32_channel3.Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(&htimer32_channel3);
}

static void Timer32_2_Init() {
    htimer32_2.Instance = TIMER32_2;
    htimer32_2.Top = 3200 - 1;
    htimer32_2.State = TIMER32_STATE_DISABLE;
    htimer32_2.Clock.Source = TIMER32_SOURCE_PRESCALER;
    htimer32_2.Clock.Prescaler = 0;
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
    husart1.baudrate = 4800;
    HAL_USART_Init(&husart1);
}