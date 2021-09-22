/** @file main.c
 * 
 * @brief
 * Main program file for ECE 443 Project 3 using FreeRTOS V202104.00 
 *
 * @details       
 * Demonstrates the use of FreeRTOS task scheduling and interrupt handling.
 * Receives messages from UART1, stores them in an I2C EEPROM, retrieves them
 * from the EEPROM, and prints them to an LCD. Uses the UART1 RX interrupt to
 * receive individual characters from UART1. Passes them through a queue to a
 * task, which stores them in the EEPROM. When BTN1 is pressed, another task
 * will read from the EEPROM and display the message on the LCD, breaking lines
 * in nice places and scrolling upward. LEDA and LEDB indicate whether the
 * EEPROM can be written to or read from, and LEDC acts as a "heartbeat" to
 * assess timings.
 *
 * @author
 * Parker Piedmont
 * 
 * @date
 * 21 Sep 2021
 */



/* INCLUDES ================================================================= */

// Kernel includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Hardware specific includes 
#include "CerebotMX7cK.h"  // JFF

// PIC32 peripheral library
#include <plib.h>

// user-defined includes
#include "comm.h"
#include "LCDlib.h"
#include "EEPROMlib.h"



/* CONSTANTS ================================================================ */

// I2C configs
#define Fsck        	400000
#define BRG_VAL     	((FPB / 2 / Fsck) - 2)
#define MAX_MSG_LEN   	80

// EEPROM configs
#define SLAVE_ADDRESS   	0x50
#define EEPROM_BASE_ADDR    0x10
#define EEPROM_OFFSET       0x20  // treated as sort of a circular buffer
#define EEPROM_MAX_MSGS     5



/* FUNCTION PROTOTYPES ====================================================== */

// tasks
static void printToLCD(void* pvParameters);
static void writeToEEPROM(void* pvParameters);
static void toggleLEDC(void* pvParameters);
static void isEEPROMFull(void* pvParameters);

// ISRs
void __attribute__( (interrupt(ipl1),
					 vector(_CHANGE_NOTICE_VECTOR))) CN_ISR_handler( void );
void __attribute__( (interrupt(ipl1),
					 vector(_UART_1_VECTOR))) U1RX_ISR_handler( void );

// hardware setup
static void prvSetupHardware( void );
void PMP_init(void);
void cn_interrupt_initialize(void);
void uart_rx_interrupt_initialize(void);



/* GLOBAL VARIABLES ========================================================= */

// inter-task communication
xQueueHandle UartRxQueue;
xSemaphoreHandle unblockPrintToLCD;

// EEPROM status
int eeprom_free_space;



/* TRACE STRINGS ============================================================ */
    
#if ( configUSE_TRACE_FACILITY == 1 )
    traceString cn_isr_trace;
    traceString uart_isr_trace;
    traceString int_trace;
    traceString eeprom_trace;
    traceString lcd_trace;
#endif



/* MAIN ===================================================================== */

/* main Function Description ***************************************************
 * SYNTAX:			int main(void);
 * KEYWORDS:		Initialize, create, tasks, scheduler
 * DESCRIPTION:		This is a typical RTOS set up function. Hardware is
 * 					initialized, tasks are created, and the scheduler is
 * 					started.
 * PARAMETERS:		None
 * RETURN VALUE:	Exit code - used for error handling
 * NOTES:			None
 * END DESCRIPTION ************************************************************/
int main(void)
{
	// configure hardware
    prvSetupHardware();
    
    // initialize tracealyzer and start recording
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTraceEnable(TRC_START);
        cn_isr_trace = xTraceRegisterString("CN ISR");
        uart_isr_trace = xTraceRegisterString("UART RX ISR");
        int_trace = xTraceRegisterString("Interrupts");
        eeprom_trace = xTraceRegisterString("EEPROM");
        lcd_trace = xTraceRegisterString("LCD");
    #endif

	// EEPROM starts empty
    eeprom_free_space = EEPROM_MAX_MSGS;

	// must occur before LCD can be written to
    LCD_init();
    
	// initialize inter-task communication
    UartRxQueue = xQueueCreate(MAX_MSG_LEN+1, sizeof(char));
    unblockPrintToLCD = xSemaphoreCreateCounting(5, 0);
    
    if (UartRxQueue != NULL && unblockPrintToLCD != NULL)
    {
        // create tasks and start scheduler
		xTaskCreate(toggleLEDC, "Toggle LEDC", configMINIMAL_STACK_SIZE,
					NULL, 3, NULL);
        xTaskCreate(printToLCD, "LCD Print", configMINIMAL_STACK_SIZE,
					NULL, 2, NULL);
        xTaskCreate(writeToEEPROM, "EEPROM Write", configMINIMAL_STACK_SIZE,
					NULL, 1, NULL);
        xTaskCreate(isEEPROMFull, "EEPROM Status", configMINIMAL_STACK_SIZE,
					NULL, 1, NULL);

        vTaskStartScheduler();
    }

    // return error if there isn't enough memory for the heap
	// or if a variable fails to initialize
    return 1;
}



/* TASK DEFINITIONS ========================================================= */

/* printToLCD Function Description *********************************************
 * SYNTAX:          static void printToLCD(void *pvParameters)
 * KEYWORDS:        LCD, EEPROM, read, I2C, BTN1, counting semaphore
 * DESCRIPTION:     Reads a message from the EEPROM and displays it on the LCD.
 * 					Breaks lines in appropriate places and scrolls upward one
 * 					line per second.
 * PARAMETER 1:     void pointer - unused
 * RETURN VALUE:    None (There is no returning from this function)
 * NOTES:           Normally blocked, but will become
 * 					unblocked when the CN ISR gives it a semaphore.
 * END DESCRIPTION ************************************************************/
static void printToLCD(void* pvParameters)
{
	// make sure semaphore is empty before doing anything
    xSemaphoreTake(unblockPrintToLCD, 0);
    
	// message to be loaded from EEPROM
    char message[MAX_MSG_LEN+1];

	// used to calculate address to read from
    int eeprom_index_r = 0;
    
	// used to implement scrolling in LCD
    char LCD_line1[16] = "                ";
    char LCD_line2[16] = "                ";
    char LCD_str[33];
    
	// used to break lines in convenient places
    int start = 0;
    int end = 0;
    int line_count = 0;
    
	// initialize message to null chars
    int i;
    for (i = 0; i < MAX_MSG_LEN+1; i++)
        message[i] = 0;
    
	// initialize string to print to LCD as spaces (null-terminated)
    for (i = 0; i < 32; i++)
        LCD_str[i] = ' ';
    LCD_str[32] = 0;
    
    while (1)
    {
        // attempt to take semaphore, block for as long as possible
        xSemaphoreTake(unblockPrintToLCD, portMAX_DELAY);
        
        // wait for press and debounce
        while(!PORTReadBits(IOPORT_G, BTN1));
        vTaskDelay(20 / portTICK_RATE_MS);
        
        // wait for release and debounce
        while(PORTReadBits(IOPORT_G, BTN1));
        vTaskDelay(20 / portTICK_RATE_MS);
        
        // clear CN interrupt flag after button has stopped bouncing
        PORTRead(IOPORT_G);
        mCNClearIntFlag();
        
        #if ( configUSE_TRACE_FACILITY == 1 )
            vTracePrint(int_trace, "Cleared CN int flag");
        #endif
        
        // re-enable interrupt
        mCNIntEnable(1);
        
        #if ( configUSE_TRACE_FACILITY == 1 )
            vTracePrint(int_trace, "Re-enabled CN interrupt");
        #endif
        
        // print if a message is available to print
        if (eeprom_free_space < EEPROM_MAX_MSGS)  // if EEPROM is not empty
        {
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(eeprom_trace, "EEPROM not empty");
            #endif

			// load message from EEPROM
            LATBCLR = LEDA;
            I2CReadEEPROM(SLAVE_ADDRESS, 
						  EEPROM_BASE_ADDR + (EEPROM_OFFSET * eeprom_index_r),
						  message, MAX_MSG_LEN+1);
            LATBSET = LEDA;
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(eeprom_trace, "Read from EEPROM");
            #endif

			// "index" of message rolls over to implement circular buffer
            eeprom_index_r++;
            eeprom_index_r %= EEPROM_MAX_MSGS;

			// free up one slot in the EEPROM
            eeprom_free_space++;

			// print to LCD, breaking lines in nice places
            i = 0;
            while (1)
            {
				// find last space before line break
                while (message[i] != 0 && line_count < 17)
                {
                    if (message[i] == ' ')
                        end = i;

                    i++;
                    line_count++;
                }

				// don't forget last word
                if (message[i] == 0)
                    end = i;
                
                if (end <= i-16)  // if there are no spaces in the line
                    end = i - 1;  // don't cut of last character

				// copy nicely broken line to a buffer filled with spaces
                strncpy(LCD_line2, message+start, end-start);

				// concatenate buffers (null-terminated)
                strncpy(LCD_str+0, LCD_line1, 16);
                strncpy(LCD_str+16, LCD_line2, 16);

				// update LCD
                LCD_clear();
                LCD_puts(LCD_str);
                
                #if ( configUSE_TRACE_FACILITY == 1 )
                    vTracePrint(lcd_trace, "Wrote to LCD");
                #endif

				// move line 2 up to line 1 and reset line 2
                strncpy(LCD_line1, LCD_line2, 16);
                strncpy(LCD_line2, "                ", 16);

                if (message[i] == 0)
                    break;
                
				// move on to next nicely broken line
                i = end;
                start = end;
                line_count = 0;

                vTaskDelay(1000 / portTICK_RATE_MS);
            }

            vTaskDelay(1000 / portTICK_RATE_MS);

			// two lines will be filled after reaching null terminator
			// scroll them up one
            strncpy(LCD_str+0, LCD_line1, 16);
            strncpy(LCD_str+16, LCD_line2, 16);

            LCD_clear();
            LCD_puts(LCD_str);
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(lcd_trace, "Wrote to LCD");
            #endif

			// clear last filled line
            vTaskDelay(1000 / portTICK_RATE_MS);
            LCD_clear();
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(lcd_trace, "Cleared LCD");
            #endif

			// reset strings for next push to LCD
            strncpy(LCD_line1, "                ", 16);
            strncpy(LCD_line2, "                ", 16);

            vTaskDelay(1000 / portTICK_RATE_MS);
        }
        else
        {
            putsU1("No more messages to display - EEPROM is empty");
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(eeprom_trace, "EEPROM empty");
            #endif
        }
    }
}

/* writeToEEPROM Function Description ******************************************
 * SYNTAX:          static void writeToEEPROM(void *pvParameters)
 * KEYWORDS:        EEPROM, write, I2C
 * DESCRIPTION:     Receives a message one character at a time through a queue.
 * 					Constructs a string one character at a time, terminating
 * 					when \r is detected. Writes the string to an EEPROM,
 * 					replacing \r with a null terminator.
 * PARAMETER 1:     void pointer - unused
 * RETURN VALUE:    None (There is no returning from this function)
 * NOTES:           Is not "synchronized" to UART RX ISR, unlike printToLCD
 * END DESCRIPTION ************************************************************/
static void writeToEEPROM(void* pvParameters)
{
	// message to store in EEPROM
    char message[MAX_MSG_LEN+1];

	// used to calculate address to write to
    int eeprom_index_w = 0;
    
	// used to build message string
    char rx = 0;
    int msg_index = 0;

	// check whether '\r' has been received
    int terminated = 0;
    
	// initialize message to null chars
    int i;
    for (i = 0; i < MAX_MSG_LEN+1; i++)
        message[i] = 0;

	// check if queue receive worked
	portBASE_TYPE xStatus;
    
    while (1)
    {
		// build string (not to exceed maximum string length)
        while (!terminated && msg_index < MAX_MSG_LEN)
        {
            if (uxQueueMessagesWaiting(UartRxQueue) > 0)
            {
				// character sent from UART RX ISR
                xStatus = xQueueReceive(UartRxQueue, &rx, 0);
                
				if (xStatus == pdPASS)
				{
					// add chars to message string
					if (rx != '\r')
					{
						message[msg_index] = rx;
						msg_index++;
					}
					else
					{
						terminated = 1;
					}
				}
            }
        }
        
        if (eeprom_free_space > 0)  // EEPROM is not full
        {
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(eeprom_trace, "EEPROM not full");
            #endif

			// write to EEPROM
            LATBCLR = LEDA;
            I2CWriteEEPROM(SLAVE_ADDRESS,
						   EEPROM_BASE_ADDR + (EEPROM_OFFSET * eeprom_index_w),
						   message, MAX_MSG_LEN+1);
            LATBSET = LEDA;
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(eeprom_trace, "Wrote to EEPROM");
            #endif
            
			// "index" of message rolls over to implement circular buffer
            eeprom_index_w++;
            eeprom_index_w %= EEPROM_MAX_MSGS;
            
			// consume one slot in EEPROM
            eeprom_free_space--;
        }
        else
        {
            putsU1("Cannot save new message - EEPROM is full");
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(eeprom_trace, "EEPROM full");
            #endif
        }
        
		// re-initialize variables for next iteration
        for (i = 0; i < MAX_MSG_LEN+1; i++)
            message[i] = 0;
        
        msg_index = 0;
        terminated = 0;
        rx = 0;
    }
}

/* toggleLEDC Function Description *********************************************
 * SYNTAX:          static void toggleLEDC(void *pvParameters)
 * KEYWORDS:        LEDC, toggle
 * DESCRIPTION:     Toggles LEDC every millisecond. Used as a "heartbeat" to
 * 					assess timing characteristics.
 * PARAMETER 1:     void pointer - unused
 * RETURN VALUE:    None (There is no returning from this function)
 * NOTES:           None
 * END DESCRIPTION ************************************************************/
static void toggleLEDC(void* pvParameters)
{
    while (1)
    {
        LATBINV = LEDC;
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}

/* isEEPROMFull Function Description *******************************************
 * SYNTAX:          static void isEEPROMFull(void *pvParameters)
 * KEYWORDS:        EEPROM, LEDA, LEDB
 * DESCRIPTION:     Lights LEDA if EEPROM is not full. Lights LEDB if EEPROM is
 * 					empty.
 * PARAMETER 1:     void pointer - unused
 * RETURN VALUE:    None (There is no returning from this function)
 * NOTES:           None
 * END DESCRIPTION ************************************************************/
static void isEEPROMFull(void* pvParameters)
{
    while(1)
    {
        if (eeprom_free_space > 0)  // if EEPROM is not full
            LATBSET = LEDA;
        else
            LATBCLR = LEDA;
        
        if (eeprom_free_space == EEPROM_MAX_MSGS)  // if EEPROM is empty
            LATBSET = LEDB;
        else
            LATBCLR = LEDB;
    }
}



/* INTERRUPT SERVICE ROUTINES =============================================== */

/* CN_ISR_handler Function Description *****************************************
 * SYNTAX:          void CN_ISR_handler(void)
 * KEYWORDS:        ISR, CN, semaphore
 * DESCRIPTION:     Gives a semaphore that unblocks printToLCD. Lights LEDD
 *                  while this function is running. Forces a context switch to
 *                  printToLCD rather than to the task that was interrupted.
 * RETURN VALUE:    None
 * NOTES:           Invoked by an assembly wrapper defined in cn_isr_wrapper.S.
 *                  This function's name must be defined in the assembly file
 *                  using .extern.
 * END DESCRIPTION ************************************************************/
void CN_ISR_handler(void)
{
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(cn_isr_trace, "Entered C portion");
    #endif

    // disable interrupt - will be re-enabled by printToLCD
    mCNIntEnable(0);
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(int_trace, "Disabled CN interrupt");
    #endif

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    LATBSET = LEDD;
    
    // give semaphore to unblock printToLCD
	// counting semaphore allows multiple button presses to be queued
    xSemaphoreGiveFromISR(unblockPrintToLCD, &xHigherPriorityTaskWoken);
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(cn_isr_trace, "Gave semaphore");
    #endif

    LATBCLR = LEDD;
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(cn_isr_trace, "Exiting");
    #endif
    
    // switch context to higher priority task (toggleLEDC)
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/* U1RX_ISR_handler Function Description ***************************************
 * SYNTAX:          void U1RX_ISR_handler(void)
 * KEYWORDS:        ISR, UART, receive
 * DESCRIPTION:     Receives a single character from UART1. Echoes the character
 * 					back through UART1 and sends it to writeToEEPROM through a
 * 					queue.
 * RETURN VALUE:    None
 * NOTES:           Invoked by an assembly wrapper defined in
 * 					u1rx_isr_wrapper.S. This function's name must be defined in
 * 					the assembly file using .extern.
 * END DESCRIPTION ************************************************************/
void U1RX_ISR_handler(void)
{
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(uart_isr_trace, "Entered C portion");
    #endif

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    
    char rx;

	// get character from UART
    getcU1(&rx);
    
	// echo to UART
    if (rx == '\r' || rx == '\n')
    {
        putcU1('\n');
        putcU1('\r');
    }
    else
    {
        putcU1(rx);
    }
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(uart_isr_trace, "Echoed to terminal");
    #endif
    
	// send character to writeToEEPROM
    xQueueSendToBackFromISR(UartRxQueue, &rx, &xHigherPriorityTaskWoken);
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(uart_isr_trace, "Sent char to queue");
    #endif
    
	// clear interrupt flag
    mU1AClearAllIntFlags();
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(int_trace, "Cleared UART int flags");
    #endif
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(uart_isr_trace, "Exiting");
    #endif
    
    // switch context to higher priority task (toggleLEDC)
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}



/* SETUP FUNCTION DEFINITIONS =============================================== */

/* prvSetupHardware Function Description ***************************************
 * SYNTAX:          static void prvSetupHardware(void)
 * KEYWORDS:        Initialize, interrupts
 * DESCRIPTION:     Sets up the Cerebot hardware and defines LEDA-LEDD as
 *                  outputs. Initializes the PMP, I2C2, and UART1. Enables the
 * 					CN and UART1 RX interrupts.
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
    
	// initialize PMP, I2C2, and UART1
    PMP_init();
    OpenI2C2(I2C_EN, BRG_VAL);
    initialize_uart1(19200, ODD_PARITY);
    
	// initialize interrupts
    cn_interrupt_initialize();
    uart_rx_interrupt_initialize();
    
    /* Enable multi-vector interrupts */
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);  /* Do only once */
    INTEnableInterrupts();   /*Do as needed for global interrupt control */
    portDISABLE_INTERRUPTS();
}

/* PMP_init Function Description ***********************************************
 * SYNTAX:          void PMP_init();
 * PARAMETER1:      No Parameters
 * KEYWORDS:        initialize, PMP
 * DESCRIPTION:     Configures the parallel master port to be able to
 * 					communicate with the LCD module.
 * RETURN VALUE:    None
 * END DESCRIPTION ************************************************************/
void PMP_init(void)
{
    int cfg1 = PMP_ON | PMP_READ_WRITE_EN | PMP_READ_POL_HI | PMP_WRITE_POL_HI;
    int cfg2 = PMP_DATA_BUS_8 | PMP_MODE_MASTER1 | PMP_WAIT_BEG_1 | 
               PMP_WAIT_MID_2 | PMP_WAIT_END_1;
    int cfg3 = PMP_PEN_0;        // only PMA0 enabled
    int cfg4 = PMP_INT_OFF;      // no interrupts used
    mPMPOpen(cfg1, cfg2, cfg3, cfg4);
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

/* uart_rx_interrupt_initialize Function Description ***************************
 * SYNTAX:          void uart_rx_interrupt_initialize();
 * PARAMETER1:      No Parameters
 * KEYWORDS:        initialize, UART receive, interrupts
 * DESCRIPTION:     Configures the UART receive interrupt. This will be
 * 					triggered any time UART1 receives data.
 * RETURN VALUE:    None
 * END DESCRIPTION ************************************************************/
void uart_rx_interrupt_initialize(void)
{
	// set priority
    mU1ASetIntPriority(1);
    mU1ASetIntSubPriority(0);

	// clear flag and enable
    mU1AClearAllIntFlags();
    mU1ARXIntEnable(1);
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

void vApplicationTickHook( void )
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}

void _general_exception_handler( unsigned long ulCause, unsigned long ulStatus )
{
	/* This overrides the definition provided by the kernel.  Other exceptions 
	should be handled here. */
	for( ;; );
}

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
