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
#include "semphr.h"

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

static void pollBTN(void* pvParameters);
static void pollBTNs(void* pvParameters);
static void BTN1Handler(void* pvParameters);
static void BTN2Handler(void* pvParameters);
static void BTN3Handler(void* pvParameters);

// control unit tasks
static void whichBTN(void* pvParameters);
static void switchMode(void* pvParameters);
static void setLowTemp(void* pvParameters);
static void setHighTemp(void* pvParameters);
static void sendCtrl(void* pvParameters);
static void requestData(void* pvParameters);

// I/O unit tasks
static void printToLCD(void* pvParameters);
static void readTemp(void* pvParameters);
static void sendPWM(void* pvParameters);
static void readSpeed(void* pvParameters);
static void sendData(void* pvParameters);

// ISRs
//void __attribute__( (interrupt(ipl2),
//					 vector(_CHANGE_NOTICE_VECTOR))) CN_ISR_handler( void );

// hardware setup
static void prvSetupHardware( void );
void PMP_init(void);



/* TASK SIGNALS ============================================================= */

//xSemaphoreHandle unblockWhichBTN;

xSemaphoreHandle unblockBTN1Handler;
xSemaphoreHandle unblockBTN2Handler;
xSemaphoreHandle unblockBTN3Handler;

xSemaphoreHandle unblockSwitchMode;
xSemaphoreHandle unblockSetLowTemp;
xSemaphoreHandle unblockSetHighTemp;



/* GLOBAL VARIABLES ========================================================= */





/* TRACE STRINGS ============================================================ */
    
#if ( configUSE_TRACE_FACILITY == 1 )
    traceString cn_intgen_trace;
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
        cn_intgen_trace = xTraceRegisterString("CN Int Gen");
        cn_isr_trace = xTraceRegisterString("CN ISR");
		heartbeat_trace = xTraceRegisterString("Heartbeat");
		read_and_save_trace = xTraceRegisterString("Read & Save");
		lcd_trace = xTraceRegisterString("LCD");
		error_trace = xTraceRegisterString("Error");
    #endif

//    LCD_init();
    unblockBTN1Handler = xSemaphoreCreateBinary();
    unblockBTN2Handler = xSemaphoreCreateBinary();
    unblockBTN3Handler = xSemaphoreCreateBinary();

	if ((unblockBTN1Handler != NULL) && (unblockBTN2Handler != NULL) && (unblockBTN3Handler != NULL))
	{
		// create tasks and start scheduler
//		xTaskCreate(printToLCD, "Print to LCD", configMINIMAL_STACK_SIZE, NULL,
//                    1, NULL);
        xTaskCreate(pollBTN, "Poll BTN1", configMINIMAL_STACK_SIZE,
                    (void*) BTN1, 2, NULL);
        xTaskCreate(pollBTN, "Poll BTN2", configMINIMAL_STACK_SIZE,
                    (void*) BTN2, 2, NULL);
        xTaskCreate(pollBTN, "Poll BTN3", configMINIMAL_STACK_SIZE,
                    (void*) BTN3, 2, NULL);
        
        xTaskCreate(BTN1Handler, "BTN1 Handler", configMINIMAL_STACK_SIZE, NULL,
                    1, NULL);
        xTaskCreate(BTN2Handler, "BTN2 Handler", configMINIMAL_STACK_SIZE, NULL,
                    1, NULL);
        xTaskCreate(BTN3Handler, "BTN3 Handler", configMINIMAL_STACK_SIZE, NULL,
                    1, NULL);

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

static void pollBTN(void* pvParameters)
{
    unsigned int btn = (unsigned int) pvParameters;
    unsigned int port;
    unsigned int btn_curr = 0;
    unsigned int btn_prev = 0;
    
    xSemaphoreHandle unblock;
    
    const TickType_t period = 1 / portTICK_RATE_MS;
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    if (btn == BTN1)
    {
        port = IOPORT_G;
        unblock = unblockBTN1Handler;
    }
    else if (btn == BTN2)
    {
        port = IOPORT_G;
        unblock = unblockBTN2Handler;
    }
    else if (btn == BTN3)
    {
        port = IOPORT_A;
        unblock = unblockBTN3Handler;
    }
    
    while (1)
    {
        btn_curr = PORTReadBits(port, btn);
        
        if (btn_curr && !btn_prev)
        {
            vTaskDelay(20 / portTICK_RATE_MS);
            
            if (PORTReadBits(port, btn))
            {
                xSemaphoreGive(unblock);
            }
        }
        
        btn_prev = btn_curr;
        
        vTaskDelayUntil(&lastWakeTime, period);
    }
}

static void pollBTNs(void* pvParameters)
{
    const TickType_t period = 1 / portTICK_RATE_MS;
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (1)
    {
        if (PORTReadBits(IOPORT_G, BTN1))
        {
            xSemaphoreGive(unblockBTN1Handler);
        }

        if (PORTReadBits(IOPORT_G, BTN2))
        {
            xSemaphoreGive(unblockBTN2Handler);
        }

        if (PORTReadBits(IOPORT_A, BTN3))
        {
            xSemaphoreGive(unblockBTN3Handler);
        }
        
        vTaskDelayUntil(&lastWakeTime, period);
    }
}

static void BTN1Handler(void* pvParameters)
{
    xSemaphoreTake(unblockBTN1Handler, 0);
    
    while (1)
    {
        xSemaphoreTake(unblockBTN1Handler, portMAX_DELAY);
        LATBINV = LEDA;
    }
}

static void BTN2Handler(void* pvParameters)
{
    xSemaphoreTake(unblockBTN2Handler, 0);
    
    while (1)
    {
        xSemaphoreTake(unblockBTN2Handler, portMAX_DELAY);
        LATBINV = LEDB;
    }
}

static void BTN3Handler(void* pvParameters)
{
    xSemaphoreTake(unblockBTN3Handler, 0);
    
    while (1)
    {
        xSemaphoreTake(unblockBTN3Handler, portMAX_DELAY);
        LATBINV = LEDC;
    }
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