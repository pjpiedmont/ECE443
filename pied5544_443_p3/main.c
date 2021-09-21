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

#include "LCDlib.h"
#include "EEPROMlib.h"

#define Fsck        400000
#define BRG_VAL     ((FPB / 2 / Fsck) - 2)
#define DATA_LEN    80

/*-----------------------------------------------------------*/

// configure hardware to run this program
static void prvSetupHardware( void );
void PMP_init(void);
void I2C2_init(void);

// 
static void printToLCD(void* pvParameters);
static void writeToEEPROM(void* pvParameters);

static void toggleLEDC(void* pvParameters);
    
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

    LCD_init();

    // create tasks and start scheduler
    xTaskCreate(printToLCD, "LCD Print", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(writeToEEPROM, "EEPROM Write", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(toggleLEDC, "Toggle LEDC", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    vTaskStartScheduler();

    // return error if there isn't enough memory for the heap
    return 1;
}

static void printToLCD(void* pvParameters)
{
    char string1[] = "Does Dr J prefer PIC32 or FPGA??";
    char string2[] = "Answer: \116\145\151\164\150\145\162\041";
    
    while (1)
    {
        LCD_puts(string1);
        vTaskDelay(5000 / portTICK_RATE_MS);
        LCD_clear();
        
        LCD_puts(string2);
        vTaskDelay(5000 / portTICK_RATE_MS);
        LCD_clear();
    }
}

static void writeToEEPROM(void* pvParameters)
{
    char slave_address = 0x50;
    char write_data[DATA_LEN], read_data[DATA_LEN];
    int mem_addr = 0x86, equal = 1;
    
    int i;
    for (i = 0; i < DATA_LEN; i++)
    {
        write_data[i] = 0;
        read_data[i] = 0;
    }
    
//    write_data = "Hello world!\r";
    strcpy(write_data, "Hello world!\r");
    
    I2CWriteEEPROM(slave_address, mem_addr, write_data, DATA_LEN);
    I2CReadEEPROM(slave_address, mem_addr, read_data, DATA_LEN);
    
    for (i = 0; i < DATA_LEN; i++)
        if (read_data[i] != write_data[i])
            equal = 0;
    
    if (equal)
        PORTSetBits(IOPORT_G, LED4);
    else
        PORTSetBits(IOPORT_G, LED1);
}

static void toggleLEDC(void* pvParameters)
{
    while (1)
    {
        LATBINV = LEDC;
        vTaskDelay(1 / portTICK_RATE_MS);
    }
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
    
    PMP_init();
    I2C2_init();
    
    /* Enable multi-vector interrupts */
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);  /* Do only once */
    INTEnableInterrupts();   /*Do as needed for global interrupt control */
    portDISABLE_INTERRUPTS();
}

void PMP_init(void)
{
    int cfg1 = PMP_ON|PMP_READ_WRITE_EN|PMP_READ_POL_HI|PMP_WRITE_POL_HI;
    int cfg2 = PMP_DATA_BUS_8 | PMP_MODE_MASTER1 | PMP_WAIT_BEG_1 | 
               PMP_WAIT_MID_2 | PMP_WAIT_END_1;
    int cfg3 = PMP_PEN_0;        // only PMA0 enabled
    int cfg4 = PMP_INT_OFF;      // no interrupts used
    mPMPOpen(cfg1, cfg2, cfg3, cfg4);
}

void I2C2_init(void)
{
//    int Fsck = 400000;
//    int BRG_VAL = ((FPB / 2 / Fsck) - 2);
    
    OpenI2C2(I2C_EN, BRG_VAL);
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
