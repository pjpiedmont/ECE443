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

#include "comm.h"
#include "LCDlib.h"
#include "EEPROMlib.h"

/*-----------------------------------------------------------*/

#define Fsck        400000
#define BRG_VAL     ((FPB / 2 / Fsck) - 2)
#define DATA_LEN    80

#define SLAVE_ADDRESS   0x50

#define EEPROM_BASE_ADDR    0x10
#define EEPROM_OFFSET       0x20
#define EEPROM_MAX_MSGS     5

/*-----------------------------------------------------------*/

// configure hardware to run this program
static void prvSetupHardware( void );
void PMP_init(void);

// enable CN interrupt
void cn_interrupt_initialize(void);

// C portion of CN ISR
void __attribute__( (interrupt(ipl1), vector(_CHANGE_NOTICE_VECTOR))) CN_ISR_handler( void );

void uart_rx_interrupt_initialize(void);
void __attribute__( (interrupt(ipl1), vector(_UART_1_VECTOR))) U1RX_ISR_handler( void );

// 
static void printToLCD(void* pvParameters);
static void writeToEEPROM(void* pvParameters);
static void toggleLEDC(void* pvParameters);
static void isEEPROMFull(void* pvParameters);

/*-----------------------------------------------------------*/

xQueueHandle UartRxQueue;
xSemaphoreHandle unblockPrintToLCD;

int eeprom_free_space;

/*-----------------------------------------------------------*/
    
#if ( configUSE_TRACE_FACILITY == 1 )
    traceString cn_isr_trace;
    traceString uart_isr_trace;
    traceString int_trace;
    traceString eeprom_trace;
    traceString lcd_trace;
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
        cn_isr_trace = xTraceRegisterString("CN ISR");
        uart_isr_trace = xTraceRegisterString("UART RX ISR");
        int_trace = xTraceRegisterString("Interrupts");
        eeprom_trace = xTraceRegisterString("EEPROM");
        lcd_trace = xTraceRegisterString("LCD");
    #endif

    eeprom_free_space = EEPROM_MAX_MSGS;

    LCD_init();
    
    UartRxQueue = xQueueCreate(DATA_LEN+1, sizeof(char));
    unblockPrintToLCD = xSemaphoreCreateCounting(5, 0);
    
    if (UartRxQueue != NULL && unblockPrintToLCD != NULL)
    {
        // create tasks and start scheduler
        xTaskCreate(printToLCD, "LCD Print", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
        xTaskCreate(writeToEEPROM, "EEPROM Write", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
        xTaskCreate(isEEPROMFull, "EEPROM Status", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
        xTaskCreate(toggleLEDC, "Toggle LEDC", configMINIMAL_STACK_SIZE, NULL, 3, NULL);

        vTaskStartScheduler();
    }

    // return error if there isn't enough memory for the heap
    return 1;
}

static void printToLCD(void* pvParameters)
{
    xSemaphoreTake(unblockPrintToLCD, 0);
    
    char message[DATA_LEN+1];
    int eeprom_index_r = 0;
    
    unsigned int btn = 0;
    unsigned int btn_prev = 0;
    
    char LCD_line1[16] = "                ";
    char LCD_line2[16] = "                ";
    char LCD_str[33];
    
    int start = 0;
    int end = 0;
    int line_count = 0;
    
    int i;
    for (i = 0; i < DATA_LEN+1; i++)
        message[i] = 0;
    
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

            LATBCLR = LEDA;
            I2CReadEEPROM(SLAVE_ADDRESS, EEPROM_BASE_ADDR + (EEPROM_OFFSET * eeprom_index_r), message, DATA_LEN+1);
            LATBSET = LEDA;
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(eeprom_trace, "Read from EEPROM");
            #endif

            eeprom_index_r++;
            eeprom_index_r %= EEPROM_MAX_MSGS;

            eeprom_free_space++;

            i = 0;
            while (1)
            {
                while (message[i] != 0 && line_count < 17)
                {
                    if (message[i] == ' ')
                        end = i;

                    i++;
                    line_count++;
                }

                if (message[i] == 0)
                    end = i;
                
                if (end <= i-16)
                    end = i - 1;

                strncpy(LCD_line2, message+start, end-start);

                strncpy(LCD_str+0, LCD_line1, 16);
                strncpy(LCD_str+16, LCD_line2, 16);

                LCD_clear();
                LCD_puts(LCD_str);
                
                #if ( configUSE_TRACE_FACILITY == 1 )
                    vTracePrint(lcd_trace, "Wrote to LCD");
                #endif

                strncpy(LCD_line1, LCD_line2, 16);
                strncpy(LCD_line2, "                ", 16);

                if (message[i] == 0)
                    break;
                
                i = end;
                start = end;
                
                line_count = 0;

                vTaskDelay(1000 / portTICK_RATE_MS);
            }

            vTaskDelay(1000 / portTICK_RATE_MS);

            strncpy(LCD_str+0, LCD_line1, 16);
            strncpy(LCD_str+16, LCD_line2, 16);

            LCD_clear();
            LCD_puts(LCD_str);
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(lcd_trace, "Wrote to LCD");
            #endif

            vTaskDelay(1000 / portTICK_RATE_MS);
            LCD_clear();
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(lcd_trace, "Cleared LCD");
            #endif

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

static void writeToEEPROM(void* pvParameters)
{
    char message[DATA_LEN+1];
    int eeprom_index_w = 0;
    
    char rx = 0;
    int msg_index = 0;
    int terminated = 0;
    
    int i;
    for (i = 0; i < DATA_LEN+1; i++)
        message[i] = 0;
    
    while (1)
    {
        while (!terminated && msg_index < DATA_LEN)
        {
            if (uxQueueMessagesWaiting(UartRxQueue) > 0)
            {
                xQueueReceive(UartRxQueue, &rx, 0);
                
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
        
        if (eeprom_free_space > 0)
        {
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(eeprom_trace, "EEPROM not full");
            #endif

            LATBCLR = LEDA;
            I2CWriteEEPROM(SLAVE_ADDRESS, EEPROM_BASE_ADDR + (EEPROM_OFFSET * eeprom_index_w), message, DATA_LEN+1);
            LATBSET = LEDA;
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(eeprom_trace, "Wrote to EEPROM");
            #endif
            
            eeprom_index_w++;
            eeprom_index_w %= EEPROM_MAX_MSGS;
            
            eeprom_free_space--;
        }
        else
        {
            putsU1("Cannot save new message - EEPROM is full");
            
            #if ( configUSE_TRACE_FACILITY == 1 )
                vTracePrint(eeprom_trace, "EEPROM full");
            #endif
        }
        
        for (i = 0; i < DATA_LEN+1; i++)
            message[i] = 0;
        
        msg_index = 0;
        terminated = 0;
        rx = 0;
    }
}

static void toggleLEDC(void* pvParameters)
{
    while (1)
    {
        LATBINV = LEDC;
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}

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

void U1RX_ISR_handler(void)
{
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(uart_isr_trace, "Entered C portion");
    #endif

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    
    char rx;
    getcU1(&rx);
    
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
    
    xQueueSendToBackFromISR(UartRxQueue, &rx, &xHigherPriorityTaskWoken);
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(uart_isr_trace, "Sent char to queue");
    #endif
    
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
    OpenI2C2(I2C_EN, BRG_VAL);
    initialize_uart1(19200, ODD_PARITY);
    
    cn_interrupt_initialize();
    uart_rx_interrupt_initialize();
    
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

void uart_rx_interrupt_initialize(void)
{
    mU1ASetIntPriority(1);
    mU1ASetIntSubPriority(0);
    mU1AClearAllIntFlags();
    mU1ARXIntEnable(1);
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
