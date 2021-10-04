/** @file main.c
 * 
 * @brief
 * Main program file for ECE 443 Project 4 using FreeRTOS V202104.00 
 *
 * @details       
 * Demonstrates the use of SMBus, direct task notifications, and message
 * buffers. Simulates a change notice interrupt every 6 ms, unblocking a task
 * that reads from an IR sensor over SMBus. Calculates the temperature and sends
 * it through a message buffer to a task that prints it to the LCD.
 *
 * @author
 * Parker Piedmont
 * 
 * @date
 * 04 Oct 2021
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
#define FSCK  80000
const int BRG_VAL = ((FPB / 2 / FSCK) - 2);

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
    traceString cn_isr_trace;
	traceString heartbeat_trace;
	traceString read_and_save_trace;
	traceString lcd_trace;
	traceString error_trace;
#endif



/* MAIN ===================================================================== */

int main( void )
{
    prvSetupHardware();		/*  Configure hardware */
    
    // initialize tracealyzer and start recording
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTraceEnable(TRC_START);
        cn_isr_trace = xTraceRegisterString("CN ISR");
		heartbeat_trace = xTraceRegisterString("Heartbeat");
		read_and_save_trace = xTraceRegisterString("Read & Save");
		lcd_trace = xTraceRegisterString("LCD");
		error_trace = xTraceRegisterString("Error");
    #endif

    LCD_init();
    tempBuffer = xMessageBufferCreate(24);

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
		#if ( configUSE_TRACE_FACILITY == 1 )
			vTracePrint(error_trace, "Heap allocation or message buffer\
									  creation failed");
		#endif
	}

    return 1;
}



/* TASK DEFINITIONS ========================================================= */

/*!
 * @brief
 * Generates a virtual change notice interrupt every 6 ms by setting the CN
 * interrupt flag.
 * 
 * @param[in] pvParameters  Unused but required by FreeRTOS
 * 
 * @return None
 */
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

/*!
 * @brief
 * Reads the temperature from an IR sensor using SMBus and sends it to a message
 * buffer.
 * 
 * @param[in] pvParameters  Unused but required by FreeRTOS
 * 
 * @return None
 */
static void readAndSaveTemperature(void* pvParameters)
{
	// SMBus parameters
    const uint8_t slave_addr = 0x5a;
    const uint8_t command = 0x07;
    uint8_t ir_data[3];
    const int data_len = 3;
    
	// used to calculate temperature in deg C
    uint16_t temp_2_bytes;
    float kelvin;
    float celsius;
    char temp_str[20];
    
    int i = 0;
    
    uint32_t str_addr = (uint32_t) temp_str;
    uint8_t str_addr_bytes[4];
    
    str_addr_bytes[0] = str_addr & 0xff;
    str_addr_bytes[1] = (str_addr >> 8) & 0xff;
    str_addr_bytes[2] = (str_addr >> 16) & 0xff;
    str_addr_bytes[3] = (str_addr >> 24) & 0xff;
    i++;
    
	// for error checking
    size_t xBytesSent;
    
	// save task handle for notifications from ISR
	readTaskHandle = xTaskGetCurrentTaskHandle();

	while (1)
	{
		// receive notification from CN ISR
		ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);

		#if ( configUSE_TRACE_FACILITY == 1 )
			vTracePrint(read_and_save_trace, "Unblocked");
			vTracePrint(read_and_save_trace, "Reading IR sensor");
		#endif
        
        I2C1_IR_Read(slave_addr, command, ir_data, data_len);

		#if ( configUSE_TRACE_FACILITY == 1 )
			vTracePrint(read_and_save_trace, "Read IR sensor");
			vTracePrint(read_and_save_trace, "Calculating temperature");
		#endif

		// assemble bytes from IR sensor into a single integer
		// IR sensor sends temp as 2 bytes in little-endian order
        temp_2_bytes = (ir_data[1] << 8) | ir_data[0];
        
        // calculate temperature
        if (temp_2_bytes > 0x7fff)  // sensor error
        {
            sprintf(temp_str, "Temp = xxx.xx C");
        }
        else
        {
			// convert temp from binary to deg C
            kelvin = temp_2_bytes * 0.02f;
            celsius = kelvin - 273.15f;
            sprintf(temp_str, "Temp = %3.2f C", celsius);
        }

		#if ( configUSE_TRACE_FACILITY == 1 )
			vTracePrint(read_and_save_trace, "Calculated temperature");
			vTracePrint(read_and_save_trace, "Sending to message buffer");
		#endif
        
		// send pointer to temp_str - not the contents of the string
        xBytesSent = xMessageBufferSend(tempBuffer, (void*)str_addr_bytes, 4, 0);

		#if ( configUSE_TRACE_FACILITY == 1 )
			vTracePrint(read_and_save_trace, "Sent to message buffer");
		#endif
        
        if (xBytesSent != sizeof(char*))
        {
            #if ( configUSE_TRACE_FACILITY == 1 )
				vTracePrint(error_trace, "Incorrect number of bytes written\
										  to message buffer");
			#endif
        }
	}
}

/*!
 * @brief
 * Reads the temperature from a message buffer and prints it to an LCD.
 * Temperature is passed using a string pointer.
 * 
 * @param[in] pvParameters  Unused but required by FreeRTOS
 * 
 * @return None
 */
static void printToLCD(void* pvParameters)
{
    char* temp_str;
    
    uint32_t str_addr = 0x00000000;
    uint8_t str_addr_bytes[4];
    
    size_t xReceivedBytes;
    
    int i = 0;

	while (1)
	{
		#if ( configUSE_TRACE_FACILITY == 1 )
			vTracePrint(lcd_trace, "Receiving from message buffer");
		#endif

        xReceivedBytes = xMessageBufferReceive(tempBuffer, (void*)str_addr_bytes, 4, 0);
        
        str_addr |= str_addr_bytes[0];
        str_addr |= (str_addr_bytes[1] << 8);
        str_addr |= (str_addr_bytes[2] << 16);
        str_addr |= (str_addr_bytes[3] << 24);
        
        temp_str = (char*) str_addr;
        i++;

		#if ( configUSE_TRACE_FACILITY == 1 )
			vTracePrint(read_and_save_trace, "Received from message buffer");
			vTracePrint(read_and_save_trace, "Printing to LCD");
		#endif

		LCD_clear();
		LCD_puts(temp_str);
        
        // LCD is illegible if it updates too quickly
        vTaskDelay(100 / portTICK_RATE_MS);

		#if ( configUSE_TRACE_FACILITY == 1 )
			vTracePrint(read_and_save_trace, "Printed to LCD");
		#endif
	}
}

/*!
 * @brief
 * Toggles LEDA every 3 ms.
 * 
 * @param[in] pvParameters  Unused but required by FreeRTOS
 * 
 * @return None
 */
static void toggleLEDA(void* pvParameters)
{
	while (1)
    {
        LATBINV = LEDA;

		#if ( configUSE_TRACE_FACILITY == 1 )
			vTracePrint(heartbeat_trace, "Toggled LEDA");
		#endif

        vTaskDelay(3 / portTICK_RATE_MS);
    }
}



/* INTERRUPT SERVICE ROUTINES =============================================== */

/*!
 * @brief
 * Unblocks readAndSaveTemperature when the CN interrupt is triggered.
 * 
 * @return None
 */
void CN_ISR_handler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	#if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(cn_isr_trace, "Starting");
    #endif
    
	if (readTaskHandle != NULL)
	{
    	vTaskNotifyGiveIndexedFromISR(readTaskHandle, 0,
                                      &xHigherPriorityTaskWoken);
	}

	#if ( configUSE_TRACE_FACILITY == 1 )
		vTracePrint(cn_isr_trace, "Notified readAndSaveTemperature");
        vTracePrint(cn_isr_trace, "Exiting");
    #endif
    
    // clear interrupt flag
    PORTRead(IOPORT_G);
    mCNClearIntFlag();
    
    // switch context to higher priority task
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}



/* SETUP FUNCTION DEFINITIONS =============================================== */

/*!
 * @brief
 * Configures the PIC32 hardware to support the operations performed by this 
 * project.
 * 
 * @return None
 */
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

/*!
 * @brief
 * Configures the parallel master port to communicate with the LCD.
 * 
 * @return None
 */
void PMP_init(void)
{
    int cfg1 = PMP_ON | PMP_READ_WRITE_EN | PMP_READ_POL_HI | PMP_WRITE_POL_HI;
    int cfg2 = PMP_DATA_BUS_8 | PMP_MODE_MASTER1 | PMP_WAIT_BEG_1 | 
               PMP_WAIT_MID_2 | PMP_WAIT_END_1;
    int cfg3 = PMP_PEN_0;        // only PMA0 enabled
    int cfg4 = PMP_INT_OFF;      // no interrupts used
    mPMPOpen(cfg1, cfg2, cfg3, cfg4);
}

/*!
 * @brief
 * Enables the CN interrupt on BTN1 at priority level 2.
 * 
 * @return None
 */
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
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(error_trace, "Exception encountered");
    #endif

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

/*** end of file ***/