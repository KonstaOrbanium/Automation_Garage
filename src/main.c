
#include "mik32_hal_spifi_w25.h"
#include "mik32_hal_timer32.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

#include "xprintf.h"

#include "Modbus.h"
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
	const led_config_t *cfg = (const led_config_t *)pvParameters;

	ModbusInit(&mHandle);

	// for (;;)
	// {
	// 	L_GPIO_TogglePin(cfg->port, cfg->pin);
		
      
    //         HAL_GPIO_TogglePin(GPIO_1, GPIO_PIN_3);
    //         HAL_GPIO_TogglePin(GPIO_0, GPIO_PIN_3);
    //         vTaskDelay(pdMS_TO_TICKS(1000));
			
	// }
	vTaskDelete((TaskHandle_t)NULL);
}


int main()
{
	HAL_Init();

	SystemClock_Config();
	// Timer32_1_Init();
	GPIO_Init();

	USART_Init();
	//xprintf("Start\n");   ///< TODO Понять, почему была ошибка

	SPIFI_Init(); 					///< Для работы с QPSIFI
	//Startup_SPIFI_Config();		///< Для работы с SPIFI
	//Мигаем несколько раз для индикации успешной инициализации.

 
	// Разрешить прерывания по уровню для линии EPIC GPIO_IRQ.

	HAL_GPIO_ClearInterrupts();
	HAL_EPIC_Clear(0xFFFFFFFF);
	
	// Создание бинарного семафора.
	xButtonSemaphore = xSemaphoreCreateBinary();
	
	// Создание задач.
	xTaskCreate(blink_task,
				"LED1",
				128,
				NULL,
				tskIDLE_PRIORITY + 1 ,
				NULL);

	vTaskStartScheduler();
}

void SystemClock_Config(void)
{
	PCC_InitTypeDef PCC_OscInit = {0};

	PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
	PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
	PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
	PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
	PCC_OscInit.AHBDivider = 0;
	PCC_OscInit.APBMDivider = 0;
	PCC_OscInit.APBPDivider = 0;
	PCC_OscInit.HSI32MCalibrationValue = 128;
	PCC_OscInit.LSI32KCalibrationValue = 8;
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
	__HAL_PCC_GPIO_IRQ_CLK_ENABLE();

	// Инициализация LED1 и LED2
	GPIO_InitStruct.Pin = LED1_PIN;
	GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
	GPIO_InitStruct.Pull = HAL_GPIO_PULL_NONE;
	HAL_GPIO_Init(LED1_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED2_PIN;
	HAL_GPIO_Init(LED2_PORT, &GPIO_InitStruct);

	// Инициализация пользовательской кнопки
	GPIO_InitStruct.Pin = BTN_PIN;
	GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_INPUT;
	HAL_GPIO_Init(BTN_PORT, &GPIO_InitStruct);
	HAL_GPIO_InitInterruptLine(BTN_IRQ_LINE_MUX, GPIO_INT_MODE_FALLING);
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

USART_HandleTypeDef husart1;
uint8_t bufPointer;
const uint8_t Modbus_FrameCount = 8;
#define BUFFER_LENGTH 50

// Обработчик прерываний.
RAM_ATTR void freertos_risc_v_application_interrupt_handler(void)
{
	if (EPIC_CHECK_GPIO_IRQ())
	{
		if (HAL_GPIO_LineInterruptState(BTN_IRQ_LINE))
		{
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;

			xSemaphoreGiveFromISR(xButtonSemaphore, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		HAL_GPIO_ClearInterrupts();
	}
 	if (EPIC_CHECK_UART_1()) {
	/* Прием данных: запись в буфер */
		if (HAL_USART_RXNE_ReadFlag(&husart1)) {
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			 
			mHandlers[0]->u8RxBuffer[bufPointer] = HAL_USART_ReadByte(&husart1);
			if (bufPointer >= Modbus_FrameCount) {
				bufPointer = 0;
				uint8_t ulNotificationValue = 0;
				
				
				if (mHandlers[0]->queueTaskSlaveHandle) {
						
					xQueueSendToBackFromISR(mHandlers[0]->queueTaskSlaveHandle, &ulNotificationValue, &xHigherPriorityTaskWoken);
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				} 
				HAL_USART_RXNE_DisableInterrupt(&husart1);
			} else {
				bufPointer += 1;
				if (bufPointer >= BUFFER_LENGTH) bufPointer = 0;              
			}
			 
			HAL_USART_RXNE_ClearFlag(&husart1);
		}             
    }
	HAL_EPIC_Clear(0xFFFFFFFF);
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
