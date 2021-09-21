/** @file main.c
 * 
 * @brief Main program file for ECE 443 Project 1 using FreeRTOS V202104.00 
 *
 * @details       
 * Demonstrates the use of FreeRTOS task scheduling. Toggles the state of LED1
 * or LED2 whenever BTN1 or BTN2, respectively, is pressed.
 *
 * @author
 * Parker Piedmont
 * @date
 * 07 Sep 2021
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Hardware specific includes. */
#include "CerebotMX7cK.h" // JFF

/* Standard demo includes. */
#include <plib.h>

/*-----------------------------------------------------------*/

// configure hardware to run this program
static void prvSetupHardware( void );

// "starter" task that starts the tasks below it
static void prvStartTasks(void *pvParameters);

// tasks to read buttons and control LEDs
static void prvSendButton(void *pvParemeters);
static void prvToggleLED(void *pvParameters);

// queues for passing button values between tasks
xQueueHandle queue1;
xQueueHandle queue2;
    
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
 * NOTES:		None
 * END DESCRIPTION *****************************************************/
int main( void )
{
    prvSetupHardware();		/*  Configure hardware */
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTraceEnable(TRC_START); // Initialize and start recording
        str = xTraceRegisterString("Channel");
    #endif

    // Create starter task then start the scheduler.
    xTaskCreate(prvStartTasks, "Task Starter", configMINIMAL_STACK_SIZE,
                NULL, 1, NULL);

    vTaskStartScheduler();

    // return error if there isn't enough memory for the heap
    return 1;
}

/* prvStartTasks Function Description ******************************************
 * SYNTAX:          static void prvStartTasks(void *pvParameters)
 * KEYWORDS:        Create, tasks, queues
 * DESCRIPTION:     This function creates two tasks that poll buttons 1
 *                  and 2 and a task that toggles LEDs 1 and 2. Also creates
 *                  two queues for communication between tasks.
 * PARAMETER 1:     void pointer - unused
 * RETURN VALUE:    None (There is no returning from this function)
 * NOTES:           None
 * END DESCRIPTION ************************************************************/
static void prvStartTasks(void *pvParameters)
{
    // create queues
    queue1 = xQueueCreate(1, sizeof(unsigned int));
    queue2 = xQueueCreate(2, sizeof(unsigned int));
    
    // start tasks that read button values
    xTaskCreate(prvSendButton, "SendButton1", configMINIMAL_STACK_SIZE,
                (void*) BTN1, 1, NULL);
    xTaskCreate(prvSendButton, "SendButton2", configMINIMAL_STACK_SIZE,
                (void*) BTN2, 1, NULL);
    
    // start task that toggles LEDs
    xTaskCreate(prvToggleLED, "ToggleLED", configMINIMAL_STACK_SIZE,
                NULL, 1, NULL);
    
    // ensure tasks have finished being created before this task deletes itself
    vTaskDelay(100 / portTICK_RATE_MS);
    
    vTaskDelete(NULL);
}

/* prvSendButton Function Description ******************************************
 * SYNTAX:          static void prvSendButton(void *pvParameters)
 * KEYWORDS:        Button, poll, queue, debounce
 * DESCRIPTION:     Checks the value of BTN1 or BTN2 and sends it to the
 *                  corresponding queue. Delays for 20ms to avoid running again
 *                  while button is bouncing.
 * PARAMETER 1:     void pointer - button bitmask
 * RETURN VALUE:    None (There is no returning from this function)
 * NOTES:           None
 * END DESCRIPTION ************************************************************/
static void prvSendButton(void *pvParameters)
{    
    // button bitmask passed through argument
    unsigned int btn = (unsigned int) pvParameters;
    unsigned int btn_on;
    
    portBASE_TYPE xStatus;
    
    while (1)
    {
        if (PORTG & btn)
            btn_on = 1;
        else
            btn_on = 0;
        
        // send button value to queue for ToggleLED task
        switch (btn)
        {
            case BTN1:
                xStatus = xQueueSendToBack(queue1, &btn_on, 0);
                #if ( configUSE_TRACE_FACILITY == 1 )
                    vTracePrint(str, "BTN1 toggled");
                #endif
                break;
                
            case BTN2:
                xStatus = xQueueSendToBack(queue2, &btn_on, 0);
                #if ( configUSE_TRACE_FACILITY == 1 )
                    vTracePrint(str, "BTN2 toggled");
                #endif
                break;
                
            default:
                #if ( configUSE_TRACE_FACILITY == 1 )
                    vTracePrint(str, "Incorrect button identifier was passed to SendButtons task");
                #endif
                break;
        }
        
        if (xStatus != pdPASS)
        {
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(str, "Could not send to the queue");
            #endif
        }
        
        // delay to account for button bounce
        vTaskDelay(20 / portTICK_RATE_MS);
        
        taskYIELD();
    }
}

/* prvToggleLED Function Description *******************************************
 * SYNTAX:          static void prvToggleLED(void *pvParameters)
 * KEYWORDS:        LED, toggle, queue, receive
 * DESCRIPTION:     Pops the front off of each button queue and compares it to
 *                  the previous value. If it changed from 0 to 1, toggle the
 *                  corresponding LED.
 * PARAMETER 1:     void pointer - unused
 * RETURN VALUE:    None (There is no returning from this function)
 * NOTES:           None
 * END DESCRIPTION ************************************************************/
static void prvToggleLED(void *pvParameters)
{
    unsigned int btn1;
    unsigned int btn1_prev = 0;
    
    unsigned int btn2;
    unsigned int btn2_prev = 0;
    
    portBASE_TYPE xStatus1;
    portBASE_TYPE xStatus2;
    
    while (1)
    {
        xStatus1 = xQueueReceive(queue1, &btn1, 0);
        xStatus2 = xQueueReceive(queue2, &btn2, 0);
        
        if (xStatus1 == pdPASS)
        {
            if (btn1 && !btn1_prev)
                LATGINV = LED1;
            
            btn1_prev = btn1;
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(str, "LED1 toggled");
            #endif
        }
        else
        {
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(str, "Failed to read from queue1");
            #endif
        }
        
        if (xStatus2 == pdPASS)
        {
            if (btn2 && !btn2_prev)
                LATGINV = LED2;

            btn2_prev = btn2;
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(str, "LED2 toggled");
            #endif
        }
        else
        {
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(str, "Failed to read from queue2");
            #endif
        }
    }
}

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
