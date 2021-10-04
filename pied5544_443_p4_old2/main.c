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



/* INCLUDES ================================================================= */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "message_buffer.h"

/* Hardware specific includes. */
#include "CerebotMX7cK.h" // JFF

/* Standard demo includes. */
#include <plib.h>

// C libraries
#include <stdio.h>
#include <stdint.h>

// user libraries
#include "LCDlib.h"
#include "IRlib.h"



/* CONSTANTS ================================================================ */

// I2C configs
const int FSCK = 400000;
const int BRG_VAL = ((FPB / 2 / 400000) - 2);

// IR sensor configs
const uint8_t SLAVE_ADDRESS = 0x5A;



/* FUNCTION PROTOTYPES ====================================================== */

// tasks
static void generateCNInt(void* pvParameters);
static void readAndSaveTemperature(void* pvParameters);
static void printToLCD(void* pvParameters);
static void toggleLEDA(void* pvParameters);

// ISRs
void __attribute__( (interrupt(ipl2),
					 vector(_CHANGE_NOTICE_VECTOR))) CN_ISR_handler( void );

// hardware setup
static void prvSetupHardware( void );
void PMP_init(void);
void cn_interrupt_initialize(void);



/* GLOBAL VARIABLES ========================================================= */

static TaskHandle_t readTaskHandle = NULL;
static MessageBufferHandle_t tempBuffer = NULL;



/* TRACE STRINGS ============================================================ */
    
#if ( configUSE_TRACE_FACILITY == 1 )
    traceString led_state_trace;
    traceString btn1_state_trace;
    traceString ledb_trace;
#endif




/* MAIN ===================================================================== */

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
        
    tempBuffer = xMessageBufferCreate(8);

	if (tempBuffer != NULL)
	{
		// create tasks and start scheduler
		xTaskCreate(generateCNInt, "Generate CN Interrupt",
					configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		xTaskCreate(readAndSaveTemperature, "Read and Save Temperature",
					configMINIMAL_STACK_SIZE, NULL, 2, NULL);
		xTaskCreate(printToLCD, "Print to LCD", configMINIMAL_STACK_SIZE, NULL,
                    1, NULL);
		xTaskCreate(toggleLEDA, "Toggle LEDA", configMINIMAL_STACK_SIZE, NULL,
                    3, NULL);

		vTaskStartScheduler();
	}

    // spin if there isn't enough memory for the heap or if message buffer
	// creation fails
	while (1)
	{
		// do nothing
	}

    return 1;
}



/* TASK DEFINITIONS ========================================================= */

static void generateCNInt(void* pvParameters)
{
	const TickType_t period = 6 / portTICK_RATE_MS;
	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1)
	{
		INTSetFlag(INT_CN);
		vTaskDelayUntil(&lastWakeTime, period);
	}
}

static void readAndSaveTemperature(void* pvParameters)
{
    const uint8_t slave_addr = 0x5A;
    const uint8_t command = 0x07;
    uint8_t ir_data[3];
    const int data_len = 3;
    
    uint16_t temp_2_bytes;
    float kelvin;
    float celsius;
    char temp_str[20];
    
    char* temp_str_ptr = &temp_str[0];
    
    size_t xBytesSent;
    
    int i = 0;
    
	readTaskHandle = xTaskGetCurrentTaskHandle();

	while (1)
	{
		ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);
        LATBINV = LEDB;
        
        I2C1_IR_Read(slave_addr, command, ir_data, data_len);
        temp_2_bytes = (ir_data[1] << 8) | ir_data[0];
        
        if (temp_2_bytes > 0x7fff)
        {
            sprintf(temp_str, "Temp = xxx.xx C");
        }
        else
        {
            kelvin = temp_2_bytes * 0.02f;
            celsius = kelvin - 273.15f;
            sprintf(temp_str, "Temp = %3.2f C", celsius);
        }
        
        xBytesSent = xMessageBufferSend(tempBuffer, (void*)temp_str_ptr, 4, 0);
        i++;
        
        if (xBytesSent != 4)
        {
            // error
        }
	}
}

static void printToLCD(void* pvParameters)
{
    char* temp_str_ptr;
	char temp_str[20];
    size_t xReceivedBytes;
    
    int i = 0;

	while (1)
	{
        xReceivedBytes = xMessageBufferReceive(tempBuffer, (void*)temp_str_ptr, 4, portMAX_DELAY);
        i++;
	}
}

static void toggleLEDA(void* pvParameters)
{
	while (1)
    {
        LATBINV = LEDA;
        vTaskDelay(3 / portTICK_RATE_MS);
    }
}



/* INTERRUPT SERVICE ROUTINES =============================================== */

void CN_ISR_handler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    LATBSET = LEDD;
    
	if (readTaskHandle != NULL)
	{
    	vTaskNotifyGiveIndexedFromISR(readTaskHandle, 0,
                                      &xHigherPriorityTaskWoken);
	}
    
    LATBCLR = LEDD;
    
    // clear interrupt flag
    PORTRead(IOPORT_G);
    mCNClearIntFlag();
    
    // switch context to higher priority task
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}




/* SETUP FUNCTION DEFINITIONS =============================================== */

static void prvSetupHardware( void )
{
    // set up Cerebot components
    Cerebot_mx7cK_setup();
    
    /* Set up PmodSTEM LEDs */
    PORTSetPinsDigitalOut(IOPORT_B, SM_LEDS);
    LATBCLR = SM_LEDS;                      /* Clear all SM LED bits */
    
    PMP_init();
    OpenI2C1(I2C_EN, BRG_VAL);
    
    // enable CN interrupt
    cn_interrupt_initialize();
    
    /* Enable multi-vector interrupts */
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);  /* Do only once */
    INTEnableInterrupts();   /*Do as needed for global interrupt control */
    portDISABLE_INTERRUPTS();
}

void PMP_init(void)
{
    int cfg1 = PMP_ON | PMP_READ_WRITE_EN | PMP_READ_POL_HI | PMP_WRITE_POL_HI;
    int cfg2 = PMP_DATA_BUS_8 | PMP_MODE_MASTER1 | PMP_WAIT_BEG_1 | 
               PMP_WAIT_MID_2 | PMP_WAIT_END_1;
    int cfg3 = PMP_PEN_0;        // only PMA0 enabled
    int cfg4 = PMP_INT_OFF;      // no interrupts used
    mPMPOpen(cfg1, cfg2, cfg3, cfg4);
}

void cn_interrupt_initialize(void)
{
    unsigned int dummy; // used to hold PORT read value
    
    // Enable CN for BTN1
    mCNOpen(CN_ON, CN8_ENABLE, 0);
    
    // Set CN interrupts priority level 1 sub priority level 0
    mCNSetIntPriority(2);       // Group priority (1 to 7)
    mCNSetIntSubPriority(0);    // Subgroup priority (0 to 3)

    // read port to clear difference
    dummy = PORTReadBits(IOPORT_G, BTN1);
    mCNClearIntFlag();          // Clear CN interrupt flag
    mCNIntEnable(1);            // Enable CNinterrupts
    
    // Global interrupts must enabled to complete the initialization.
}



/* HOOK FUNCTION DEFINITIONS ================================================ */

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
    
    while (1);
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
