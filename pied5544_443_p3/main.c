/** @file main.c
 * 
 * @brief
 * Main program file for ECE 443 Project 2 using FreeRTOS V202104.00 
 *
 * @details       
 * Demonstrates the use of FreeRTOS task scheduling and interrupt handling.
 * Indicates the state of BTN1 on LEDA. Toggles LEDB every millisecond. Toggles
 * LEDC in a push-on/push-off manner. Lights LEDD while the interrupt handler
 * is active.
 *
 * @author
 * Parker Piedmont
 * @date
 * 13 Sep 2021
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Hardware specific includes. */
#include "CerebotMX7cK.h" // JFF

/* Standard demo includes. */
#include <plib.h>

/*-----------------------------------------------------------*/

// configure hardware to run this program
static void prvSetupHardware( void );

// periodic task that will toggle LEDB
static void toggleLEDB(void* pvParameters);

// handler task that will be unblocked by an ISR
static void toggleLEDC(void* pvParameters);

// enable CN interrupt
void cn_interrupt_initialize(void);

// C portion of ISR
void __attribute__( (interrupt(ipl1), vector(_CHANGE_NOTICE_VECTOR))) CN_ISR_handler( void );

// ISR uses this to unblock toggleLEDC
xSemaphoreHandle unblockToggleLEDC;
    
#if ( configUSE_TRACE_FACILITY == 1 )
    traceString led_state_trace;
    traceString btn1_state_trace;
    traceString ledb_trace;
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
    
    // initialize tracealyzer and start recording
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTraceEnable(TRC_START);
        led_state_trace = xTraceRegisterString("LED state");
        btn1_state_trace = xTraceRegisterString("BTN1 state");
        ledb_trace = xTraceRegisterString("LEDB breakpoint");
    #endif
        
    // create binary semaphore to unblock toggleLEDC
    vSemaphoreCreateBinary(unblockToggleLEDC);
    
    if (unblockToggleLEDC != NULL)
    {   
        // create tasks and start scheduler
        xTaskCreate(toggleLEDB, "Toggle LEDB", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
        xTaskCreate(toggleLEDC, "Toggle LEDC", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

        vTaskStartScheduler();
    }

    // return error if there isn't enough memory for the heap
    // or if the semaphore failed to be created
    return 1;
}

/* toggleLEDB Function Description *********************************************
 * SYNTAX:          static void toggleLEDB(void *pvParameters)
 * KEYWORDS:        LEDB, delay, trace, task
 * DESCRIPTION:     Toggles LEDB every millisecond. While LEDC is lit, prints a
 *                  trace message after the second time this task runs.
 * PARAMETER 1:     void pointer - unused
 * RETURN VALUE:    None (There is no returning from this function)
 * NOTES:           None
 * END DESCRIPTION ************************************************************/
static void toggleLEDB(void* pvParameters)
{
    int ledb_on = 0;
    int ledc_lit_count = 0;
    
    while (1)
    {
        LATBINV = LEDB;
        
        #if ( configUSE_TRACE_FACILITY == 1 )
            ledb_on = PORTB & LEDB;
            if (ledb_on)
                vTracePrint(led_state_trace, "Toggled LEDB on");
            else
                vTracePrint(led_state_trace, "Toggled LEDB off");
        #endif
        
        if (PORTB & LEDC)
            ledc_lit_count++;
        else
            ledc_lit_count = 0;  // restart count when LEDC is not lit
        
        // print a trace message after the second time this task has run
        // while LEDC is lit
        #if ( configUSE_TRACE_FACILITY == 1 )
            if (ledc_lit_count > 2)
                vTracePrint(ledb_trace, "LEDC lit, toggleLEDB has run more than 2 times");
        #endif
        
        // delay 1ms
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}

/* toggleLEDC Function Description *********************************************
 * SYNTAX:          static void toggleLEDC(void *pvParameters)
 * KEYWORDS:        LEDC, debounce, handler, semaphore
 * DESCRIPTION:     Debounces BTN1 and toggles LEDC. Normally blocked, but can
 *                  be unblocked by giving a semaphore.
 * PARAMETER 1:     void pointer - unused
 * RETURN VALUE:    None (There is no returning from this function)
 * NOTES:           None
 * END DESCRIPTION ************************************************************/
static void toggleLEDC(void* pvParameters)
{
    // take semaphore to make sure it's cleared
    xSemaphoreTake(unblockToggleLEDC, 0);
    
    unsigned int btn = 0;
    unsigned int btn_prev = 0;
    
    int ledc_on = 0;
    
    while (1)
    {
        // attempt to take semaphore, block for as long as possible
        xSemaphoreTake(unblockToggleLEDC, portMAX_DELAY);
        vTaskDelay(20 / portTICK_RATE_MS);  // debounce
        
        btn = PORTReadBits(IOPORT_G, BTN1);
        
        if (btn && !btn_prev)  // if BTN1 was just pressed
            LATBINV = LEDC;
        
        btn_prev = btn;
        
        #if ( configUSE_TRACE_FACILITY == 1 )
            ledc_on = PORTB & LEDC;
            if (ledc_on)
                vTracePrint(led_state_trace, "Toggled LEDC on");
            else
                vTracePrint(led_state_trace, "Toggled LEDC off");
        #endif
    }
}

/* CN_ISR_handler Function Description *****************************************
 * SYNTAX:          void CN_ISR_handler(void)
 * KEYWORDS:        ISR, CN, semaphore
 * DESCRIPTION:     Gives a semaphore that unblocks toggleLEDC. Lights LEDD
 *                  while this function is running. Forces a context switch to
 *                  toggleLEDC rather than to the task that was interrupted.
 * RETURN VALUE:    None
 * NOTES:           Invoked by an assembly wrapper defined in cn_isr_wrapper.S.
 *                  This function's name must be defined in the assembly file
 *                  using .extern.
 * END DESCRIPTION ************************************************************/
void CN_ISR_handler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    LATBSET = LEDD;
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(led_state_trace, "LEDD toggled on");
    #endif
    
    // give semaphore to unblock toggleLEDC
    xSemaphoreGiveFromISR(unblockToggleLEDC, &xHigherPriorityTaskWoken);    
    LATBCLR = LEDD;
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(led_state_trace, "LEDD toggled off");
    #endif
    
    // clear interrupt flag
    PORTRead(IOPORT_G);
    mCNClearIntFlag();
    
    // switch context to higher priority task (toggleLEDC)
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/* prvSetupHardware Function Description ***************************************
 * SYNTAX:          static void prvSetupHardware(void)
 * KEYWORDS:        Initialize, interrupts
 * DESCRIPTION:     Sets up the Cerebot hardware and defines LEDA-LEDD as
 *                  outputs. Enables the CN interrupt.
 * RETURN VALUE:    None
 * NOTES:           None
 * END DESCRIPTION ************************************************************/
static void prvSetupHardware( void )
{
    // set up Cerebot components
    Cerebot_mx7cK_setup();
    
    /* Set up PmodSTEM LEDs */
    PORTSetPinsDigitalOut(IOPORT_B, SM_LEDS);
    LATBCLR = SM_LEDS;                      /* Clear all SM LED bits */
    
    // enable CN interrupt
    cn_interrupt_initialize();
    
    /* Enable multi-vector interrupts */
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);  /* Do only once */
    INTEnableInterrupts();   /*Do as needed for global interrupt control */
    portDISABLE_INTERRUPTS();
}

/* cn_interrupt_initialize Function Description ********************************
 * SYNTAX:          void cn_interrupt_initialize();
 * PARAMETER1:      No Parameters
 * KEYWORDS:        initialize, change notice, interrupts
 * DESCRIPTION:     Configures the change notice interrupt. This will occur when
 *                  BTN1 is pressed.
 * RETURN VALUE:    None
 * END DESCRIPTION ************************************************************/
void cn_interrupt_initialize(void)
{
    unsigned int dummy; // used to hold PORT read value
    
    // Enable CN for BTN1
    mCNOpen(CN_ON, CN8_ENABLE, 0);
    
    // Set CN interrupts priority level 1 sub priority level 0
    mCNSetIntPriority(1);       // Group priority (1 to 7)
    mCNSetIntSubPriority(0);    // Subgroup priority (0 to 3)

    // read port to clear difference
    dummy = PORTReadBits(IOPORT_G, BTN1);
    mCNClearIntFlag();          // Clear CN interrupt flag
    mCNIntEnable(1);            // Enable CNinterrupts
    
    // Global interrupts must enabled to complete the initialization.
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

/* vApplicationIdleHook Function Description ***********************************
 * SYNTAX:          void vApplicationIdleHook(void)
 * KEYWORDS:        LEDA, BTN1
 * DESCRIPTION:     Shows the state of BTN1 on LEDA.
 * RETURN VALUE:    None (There is no returning from this function)
 * NOTES:           Runs whenever no other task or ISR is running. Has a
 *                  priority of 0.
 * END DESCRIPTION ************************************************************/
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
    
    int count = 0;
    
    while (1)
    {
        count++;
        
        if (PORTReadBits(IOPORT_G, BTN1))
        {
            LATBSET = LEDA;
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                if (count >= 10)  // reduce frequency of messages
                {
                    vTracePrint(btn1_state_trace, "BTN1 down");
                    count = 0;
                }
            #endif
        }
        else
        {
            LATBCLR = LEDA;
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                if (count >= 10)  // reduce frequency of messages
                {
                    vTracePrint(btn1_state_trace, "BTN1 up");
                    count = 0;
                }
            #endif
        }
    }
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
