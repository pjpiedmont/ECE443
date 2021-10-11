/** @file main.c
 * 
 * @brief Main program file for Reference Design 1 using FreeRTOS V2021 
 *
 * @details       
 * Demonstrates the use of FreeRTOS, Doxygen, Git, and Tracealyzer.
 * Built on FreeRTOS V202104.00, uses CAN1 and CAN2 modules.
 * Every second, the CAN1 TX function will toggle LEDA and send the
 * LED "status" to CAN2 in an EID message. When CAN2 receives a message,
 * it will set LEDD to the value received in the message, toggle LEDB,
 * and then send the status of LEDB to CAN1. CAN1 will then set LEDC
 * to the received value.
 * 
 * @author
 * Dr J
 * @date
 * 01 Jun 2021
 */


/******************************************************************************
 * This project provides a simple CAN  project.
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Hardware specific includes. */
#include "CerebotMX7cK.h" // JFF

/* Standard demo includes. */
#include <plib.h>


#include "GenericTypeDefs.h"
#include "CANFunctions.h"

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
    
/* Create the tasks then start the scheduler. */

    /* Create the tasks defined within this file. */
    xTaskCreate( prvTestTask1, "Tsk1", configMINIMAL_STACK_SIZE,
                                    NULL, tskIDLE_PRIORITY, NULL );

    xTaskCreate( prvTestTask2, "Tsk2", configMINIMAL_STACK_SIZE,
                                    NULL, tskIDLE_PRIORITY, NULL );

    vTaskStartScheduler();	/*  Finally start the scheduler. */

/* Will only reach here if there is insufficient heap available to start
 *  the scheduler. */
    return 0;
}  /* End of main */

/* prvTestTask1 Function Description *****************************************
 * SYNTAX:          static void prvTestTask1( void *pvParameters );
 * KEYWORDS:        RTOS, Task
 * DESCRIPTION:     Every second, CAN1 toggles LEDA and sends a message
 *                  with the LED status
 * 
 * PARAMETER 1:     void pointer - data of unspecified data type sent from
 *                  RTOS scheduler
 * RETURN VALUE:    None (There is no returning from this function)
 * 
 * END DESCRIPTION ************************************************************/
static void prvTestTask1( void *pvParameters )
{

    for( ;; )
    {
        
        CAN1TxSendLEDMsg();	/* Function is defined in CANFunctions.c */
        
        #if ( configUSE_TRACE_FACILITY == 1 )
            vTracePrint(str, "CAN 1 Sent");
        #endif           
               
        vTaskDelay (1000/portTICK_PERIOD_MS);
        
    }
}  /* End of prvTestTask1 */


/* prvTestTask2 Function Description *****************************************
 * SYNTAX:          static void prvTestTask1( void *pvParameters );
 * KEYWORDS:        RTOS, Task
 * DESCRIPTION:     Checks for received messages by CAN2 and then CAN1.
 * 
 * PARAMETER 1:     void pointer - data of unspecified data type sent from
 *                  RTOS scheduler
 * RETURN VALUE:    None (There is no returning from this function)
 * 
 * END DESCRIPTION ************************************************************/
static void prvTestTask2( void *pvParameters )
{

    for( ;; )
    {      
        
        /* CAN2RxMsgProcess will check if CAN2 has received a message from CAN1 and
        * will toggle LEDD. It will send a message to CAN1 to toggle LEDB. */

         CAN2RxMsgProcess();     /* Function is defined in CANFunctions.c */

        /* CAN1RxMsgProcess() will check if CAN1  has received a message from CAN2 and
        * will toggle LEDC. */

         CAN1RxMsgProcess();     /* Function is defined in CANFunctions.c */ 

                
    }
}  /* End of prvTestTask2 */

static void prvSetupHardware( void )
{
    Cerebot_mx7cK_setup();
    
    /* Set up PmodSTEM LEDs */
    PORTSetPinsDigitalOut(IOPORT_B, SM_LEDS);
    LATBCLR = SM_LEDS;                      /* Clear all SM LED bits */
    
    /* Functions are defined in CANFunctions.c */
    CAN1Init();
    CAN2Init(); 
    
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
