/** @file main.c
 * 
 * @brief Main program file for Reference Design 1 using FreeRTOS V2021 
 *
 * @details       
 * Demonstrates the use of FreeRTOS, Doxygen, Git, and Tracealyzer.
 * Built on FreeRTOS V202104.00, design alternates between two tasks
 * which light LEDA or LEDB while turning off the other LED.
 *
 * @author
 * Dr J
 * @date
 * 01 Jun 2021
 */


/******************************************************************************
 * This project provides a simple blinky style project,
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Hardware specific includes. */
#include "CerebotMX7cK.h" // JFF

/* Standard demo includes. */
#include <plib.h>

/*-----------------------------------------------------------*/

/*
 * Set up the hardware ready to run this demo.
 */
static void prvSetupHardware( void );

/* Simple Tasks that light a specific LED when running  */
static void prvTestTask1( void *pvParameters );
static void prvTestTask2( void *pvParameters );
  
#if ( configUSE_TRACE_FACILITY == 1 )
    traceString str;
#endif

/* main Function Description ***************************************
 * SYNTAX:		int main( void );
 * KEYWORDS:		Initialize, create, tasks, scheduler
 * DESCRIPTION:         This is a typical RTOS set up function. Hardware is
 * 			initialized, tasks are created, and the scheduler is
 * 			started.
 * PARAMETERS:		None
 * RETURN VALUE:	Exit code - used for error handling
 * NOTES:		All three buttons are polled using the same code
 *                      for reading the buttons.
 * END DESCRIPTION *****************************************************/
int main( void )
{
    prvSetupHardware();		/*  Configure hardware */
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTraceEnable(TRC_START); // Initialize and start recording
        str = xTraceRegisterString("Channel");
    #endif

/* -----  NO FreeRTOS API calls BEFORE this line!!! ------------*/
    
/* Create the tasks then start the scheduler. */

    /* Create the tasks defined within this file. */
    xTaskCreate( prvTestTask1, "Tst1", configMINIMAL_STACK_SIZE,
                                    NULL, tskIDLE_PRIORITY, NULL );
    xTaskCreate( prvTestTask2, "Tst2", configMINIMAL_STACK_SIZE,
                                    NULL, tskIDLE_PRIORITY, NULL );

    vTaskStartScheduler();	/*  Finally start the scheduler. */

/* Will only reach here if there is insufficient heap available to start
 *  the scheduler. */
    return 0;
}  /* End of main */

/* prvTestTask1 Function Description *****************************************
 * SYNTAX:          static void prvTestTask1( void *pvParameters );
 * KEYWORDS:        RTOS, Task
 * DESCRIPTION:     If LEDA is not lit, all LEDs are turned off and LEDA is
 *                  turned on. Increments a counter each time the task is
 *                  resumed.
 * PARAMETER 1:     void pointer - data of unspecified data type sent from
 *                  RTOS scheduler
 * RETURN VALUE:    None (There is no returning from this function)
 * NOTES:           LEDA is switched on and LEDB switched off if LEDA was
 *                  detected as off.
 * END DESCRIPTION ************************************************************/
static void prvTestTask1( void *pvParameters )
{
unsigned int counter = 0;

    for( ;; )
    {
	if(!(LATB & LEDA))      /* Test for LEDA off */
	{
            LATBCLR = SM_LEDS;  /* Turn off all other LEDs */
            LATBSET = LEDA;     /* Turn LEDA on */
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(str, "LEDA set on");
            #endif
            ++counter;          /* Increment task run counter */
	}
    }
}  /* End of prvTestTask1 */

/* prvTestTask2 Function Description *****************************************
 * SYNTAX:          static void prvTestTask2( void *pvParameters );
 * KEYWORDS:        RTOS, Task
 * DESCRIPTION:     If LEDB is not lit, all LEDs are turned off and LEDB is
 *                  turned on. Increments a counter each time the task is
 *                  resumed.
 * PARAMETER 1:     void pointer - data of unspecified data type sent from
 *                  RTOS scheduler
 * RETURN VALUE:    None (There is no returning from this function)
 * NOTES:           LEDB is switched on and LEDA switched off if LEDB was
 *                  detected as off.
 * END DESCRIPTION ************************************************************/
static void prvTestTask2( void *pvParameters )
{
unsigned int counter = 0;
    for( ;; )
    {
	if(!(LATB & LEDB))      /* Test for LEDB off */
	{
            LATBCLR = SM_LEDS;  /* Turn off all other LEDs */
            LATBSET = LEDB;     /* Turn LEDB on */
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(str, "LEDB set on");
            #endif
            ++counter;          /* Increment task run counter */
	}
    }
}  /* End of prvTestTask2 */

static void prvSetupHardware( void )
{
    Cerebot_mx7cK_setup();
    
    /* Set up PmodSTEM LEDs */
    PORTSetPinsDigitalOut(IOPORT_B, SM_LEDS);
    LATBCLR = SM_LEDS;                      /* Clear all SM LED bits */
    LATBSET = LEDA;                         /* Turn on LEDA */
    
/* Enable multi-vector interrupts */
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);  /* Do only once */
    INTEnableInterrupts();   /*Do as needed for global interrupt control */
    portDISABLE_INTERRUPTS();
    

}

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
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time task stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook	function is 
	called if a task stack overflow is detected.  Note the system/interrupt
	stack is not checked. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/

void _general_exception_handler( unsigned long ulCause, unsigned long ulStatus )
{
	/* This overrides the definition provided by the kernel.  Other exceptions 
	should be handled here. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
volatile unsigned long ul = 0;

	( void ) pcFile;
	( void ) ulLine;

	__asm volatile( "di" );
	{
		/* Set ul to a non-zero value using the debugger to step out of this
		function. */
		while( ul == 0 )
		{
			portNOP();
		}
	}
	__asm volatile( "ei" );
}
