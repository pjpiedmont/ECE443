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

/* DATA STRUCTURES ========================================================== */

typedef struct
{
	unsigned int btn;
	unsigned int port;
	xSemaphoreHandle unblock;
} btn_struct;

/* CONSTANTS ================================================================ */

// I2C configs
#define FSCK 80000
const int BRG_VAL = ((FPB / 2 / FSCK) - 2);

// IR sensor configs
const uint8_t SLAVE_ADDRESS = 0x5A;

/* FUNCTION PROTOTYPES ====================================================== */

// control unit tasks
static void pollBTN(void *pvParameters);
static void printToLCD(void *pvParameters);
static void switchMode(void *pvParameters);
static void setTemp(void *pvParameters);
static void sendPWM(void *pvParameters);
static void requestData(void *pvParameters);

// I/O unit tasks

static void readSensors(void *pvParameters);
static void sendData(void *pvParameters);

// ISRs
//void __attribute__( (interrupt(ipl2),
//					 vector(_CHANGE_NOTICE_VECTOR))) CN_ISR_handler( void );

// hardware setup
static void prvSetupHardware(void);
void PMP_init(void);

/* TASK SIGNALS ============================================================= */

xSemaphoreHandle unblockSwitchMode;
xSemaphoreHandle unblockSetLowTemp;
xSemaphoreHandle unblockSetHighTemp;

/* GLOBAL VARIABLES ========================================================= */

uint8_t mode;

float current_temp;
float low_temp;
float high_temp;
int duty_cycle;
float rps;

/* TRACE STRINGS ============================================================ */

#if (configUSE_TRACE_FACILITY == 1)
traceString cn_intgen_trace;
traceString cn_isr_trace;
traceString heartbeat_trace;
traceString read_and_save_trace;
traceString lcd_trace;
traceString error_trace;
#endif

/* MAIN ===================================================================== */

int main(void)
{
	prvSetupHardware(); /*  Configure hardware */

// initialize tracealyzer and start recording
#if (configUSE_TRACE_FACILITY == 1)
	vTraceEnable(TRC_START);
	cn_intgen_trace = xTraceRegisterString("CN Int Gen");
	cn_isr_trace = xTraceRegisterString("CN ISR");
	heartbeat_trace = xTraceRegisterString("Heartbeat");
	read_and_save_trace = xTraceRegisterString("Read & Save");
	lcd_trace = xTraceRegisterString("LCD");
	error_trace = xTraceRegisterString("Error");
#endif

	btn_struct btn1_struct;
	btn_struct btn2_struct;
	btn_struct btn3_struct;

	btn1_struct.btn = BTN1;
	btn1_struct.port = IOPORT_G;
	btn1_struct.unblock = unblockSwitchMode;

	btn2_struct.btn = BTN2;
	btn2_struct.port = IOPORT_G;
	btn2_struct.unblock = unblockSetLowTemp;

	btn3_struct.btn = BTN3;
	btn3_struct.port = IOPORT_A;
	btn3_struct.unblock = unblockSetHighTemp;

	mode = 1;
	LATGCLR = LED1;

	low_temp = -1000;
	high_temp = 1000;

	//    LCD_init();
	unblockSwitchMode = xSemaphoreCreateBinary();
	unblockSetLowTemp = xSemaphoreCreateBinary();
	unblockSetHighTemp = xSemaphoreCreateBinary();

	if ((unblockSwitchMode != NULL) && (unblockSetLowTemp != NULL) && (unblockSetHighTemp != NULL))
	{
		// create tasks and start scheduler
		xTaskCreate(pollBTN, "BTN1 Poll", configMINIMAL_STACK_SIZE, (void *)btn1_struct, 2, NULL);
		xTaskCreate(pollBTN, "BTN2 Poll", configMINIMAL_STACK_SIZE, (void *)btn2_struct, 2, NULL);
		xTaskCreate(pollBTN, "BTN3 Poll", configMINIMAL_STACK_SIZE, (void *)btn3_struct, 2, NULL);
		xTaskCreate(printToLCD, "Print to LCD", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		xTaskCreate(switchMode, "Mode Switch", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		xTaskCreate(setLowTemp, "Low Temp Set", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		xTaskCreate(setHighTemp, "High Temp Set", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

		vTaskStartScheduler();
	}

	// spin if there isn't enough memory for the heap or if message buffer
	// creation fails
	while (1)
	{
#if (configUSE_TRACE_FACILITY == 1)
		vTracePrint(error_trace, "Heap allocation or message buffer\
									  creation failed");
#endif
	}

	return 1;
}

/* TASK DEFINITIONS ========================================================= */

static void pollBTN(void *pvParameters)
{
	btn_struct params = (btn_struct)pvParameters;

	unsigned int btn_curr = 0;
	unsigned int btn_prev = 0;

	const TickType_t period = 1 / portTICK_RATE_MS;
	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1)
	{
		btn_curr = PORTReadBits(params.port, params.btn);

		if (btn_curr && !btn_prev) // if button has been pressed
		{
			vTaskDelay(20 / portTICK_RATE_MS);

			if (PORTReadBits(params.port, params.btn))
			{
				xSemaphoreGive(params.unblock);
			}
		}

		btn_prev = btn_curr;

		vTaskDelayUntil(&lastWakeTime, period);
	}
}

static void printToLCD(void* pvParameters)
{
	char temp_curr[5];
	char temp_lo[5];
	char temp_hi[5];
	char pwm[4];
	char speed[6];

	while (1)
	{
		sprintf(temp_curr, "%2.1f", current_temp);
		sprintf(temp_lo, "%2.1f", low_temp);
		sprintf(temp_hi, "%2.1f", high_temp);
		sprintf(pwm, "%d\%", duty_cycle);
		sprintf(speed, "%3.2f", rps);

		if (!mode)  // if operational mode
		{
			LCD_clear();

			LCD_set_cursor_pos(1, 6);
			LCD_puts(temp_curr);

			LCD_set_cursor_pos(1, 0);
			LCD_puts(temp_lo);

			LCD_set_cursor_pos(1, 12);
			LCD_puts(temp_hi);

			LCD_set_cursor_pos(0, 0);
			LCD_puts(pwm);

			LCD_set_cursor_pos(0, 10);
			LCD_puts(speed);
		}
		else  // if config mode
		{
			LCD_clear();

			LCD_set_cursor_pos(0, 6);
			LCD_puts(temp_curr);

			LCD_set_cursor_pos(1, 0);
			LCD_puts(temp_lo);

			LCD_set_cursor_pos(1, 12);
			LCD_puts(temp_hi);
		}
	}
}

static void switchMode(void *pvParameters)
{
	xSemaphoreTake(unblockSwitchMode, 0);

	while (1)
	{
		xSemaphoreTake(unblockSwitchMode, portMAX_DELAY);

		// if operational mode or config mode and temps set correctly
		if (!mode || (mode && (low_temp > -1000) && (high_temp < 1000)))
		{
			// take mutex
			mode = !mode;
			// give mutex

			LATGINV = LED1;
		}
	}
}

static void setLowTemp(void *pvParameters)
{
	xSemaphoreTake(unblockSetLowTemp, 0);

	while (1)
	{
		xSemaphoreTake(unblockSetLowTemp, portMAX_DELAY);

		// send RTR frame
		// take semaphore (given by CAN1 ISR)
		// extract temp from response

		// take mutex
		// temp = can.temp;
		// give mutex

		LATBINV = LEDB;
	}
}

static void setHighTemp(void *pvParameters)
{
	xSemaphoreTake(unblockSetHighTemp, 0);

	while (1)
	{
		xSemaphoreTake(unblockSetHighTemp, portMAX_DELAY);

		// send RTR frame
		// take semaphore (given by CAN1 ISR)
		// extract temp from response

		// take mutex
		// temp = can.temp;
		// give mutex

		LATBINV = LEDB;
	}
}

static void sendPWM(void *pvParameters)
{
	while (1)
	{
		// construct data frame
		// send data frame

		LATBINV = LEDC;

		// delay 2000ms
	}
}

static void requestData(void* pvParameters)
{
	// take semaphore

	while (1)
	{
		// take semaphore
		// construct RTR frame
		// send RTR frame

		LATBINV = LEDA;
	}
}

static void readSensors(void* pvParameters)
{
	// take semaphore

	while (1)
	{
		// take semaphore
		// read 
	}
}

static void sendData(void* pvParameters)
{
	// take semaphore (given by CAN2 ISR)

	while (1)
	{
		// construct message
		// send message
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
static void prvSetupHardware(void)
{
	// set up Cerebot components
	Cerebot_mx7cK_setup();

	/* Set up PmodSTEM LEDs */
	PORTSetPinsDigitalOut(IOPORT_B, SM_LEDS);
	LATBCLR = SM_LEDS; /* Clear all SM LED bits */

	PMP_init();
	OpenI2C1(I2C_EN, BRG_VAL);

	/* Enable multi-vector interrupts */
	INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR); /* Do only once */
	INTEnableInterrupts();							   /*Do as needed for global interrupt control */
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
	int cfg3 = PMP_PEN_0;	// only PMA0 enabled
	int cfg4 = PMP_INT_OFF; // no interrupts used
	mPMPOpen(cfg1, cfg2, cfg3, cfg4);
}

/* HOOK FUNCTION DEFINITIONS ================================================ */

void vApplicationMallocFailedHook(void)
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
	for (;;)
		;
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
void vApplicationIdleHook(void)
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

	while (1)
		;
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	(void)pcTaskName;
	(void)pxTask;

	/* Run time task stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook	function is 
	called if a task stack overflow is detected.  Note the system/interrupt
	stack is not checked. */
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/

void _general_exception_handler(unsigned long ulCause, unsigned long ulStatus)
{
	/* This overrides the definition provided by the kernel.  Other exceptions 
	should be handled here. */

#if (configUSE_TRACE_FACILITY == 1)
	vTracePrint(error_trace, "Exception encountered");
#endif

	for (;;)
		;
}
/*-----------------------------------------------------------*/

void vAssertCalled(const char *pcFile, unsigned long ulLine)
{
	volatile unsigned long ul = 0;

	(void)pcFile;
	(void)ulLine;

	__asm volatile("di");
	{
		/* Set ul to a non-zero value using the debugger to step out of this
		function. */
		while (ul == 0)
		{
			portNOP();
		}
	}
	__asm volatile("ei");
}

/*** end of file ***/