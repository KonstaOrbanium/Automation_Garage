/*
 * Copyright (C) 2023, Syntacore Ltd.
 * All Rights Reserved.
 */

/*
*  SCRx FreeRTOS Demo
*  @copyright Copyright (C) 2019, Syntacore Ltd.
*  All Rights Reserved.
*  @brief FreeRTOS SCR-specific handlers and more
*/

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#include "xprintf.h"

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS();
    xprintf("\nApplication Malloc Failed!\n");
    vTaskEndScheduler();
}
/*-----------------------------------------------------------*/

void vAssertCalled( void )
{
    register unsigned long ra asm ("ra");

    taskDISABLE_INTERRUPTS();
    xprintf("\nvAssertCalled (ret addr= 0x%lx)!\n", ra);

    vTaskEndScheduler();
}

void print_exception(uint32_t cause)
{
	portDISABLE_INTERRUPTS();
	uint32_t mepc = read_csr(mepc);
	uint32_t mtval = read_csr(mtval);
	xprintf("cause: 0x%08x\n", cause);
	if (cause == 0x00000001)
	{
		xprintf("instruction access fault\n");
	}
	else if (cause == 0x00000002)
	{
		xprintf("illegal instruction\n");
	}
	xprintf("pc 0x%08x\n", mepc);
	xprintf("inst 0x%08x\n", mtval);

	uint32_t sp_reg;
	asm volatile(
		"add %0, x0, sp"
		"\n\t"
		: "=r"(sp_reg));
	xprintf("SP = 0x%08X\n", sp_reg);

	while (1)
	{
	}
}

// Обработчик исключений.
void freertos_risc_v_application_exception_handler(void)
{
	uint32_t cause = read_csr(mcause);
	if (!(cause & 0x80000000))
	{
		print_exception(cause);
	}
}