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
#include "queue.h"

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
#include "CANFunctions.h"
#include "GenericTypeDefs.h"



/* DATA STRUCTURES ========================================================== */

typedef struct
{
	unsigned int btn;
	unsigned int port;
	xSemaphoreHandle unblock;
} poll_btn_args_t;

typedef struct
{
    QueueHandle_t queue;
    xSemaphoreHandle unblock;
} set_temp_args_t;

typedef union
{
    uint64_t data;
    
    struct
    {
        uint16_t temp;
        uint8_t pwm;
        float rps;
    };
} CAN_data_t;



/* CONSTANTS ================================================================ */

#define OPERATE 1
#define CONFIG  0

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
static void CAN1RXHandler(void* pvParameters);

// control unit functions
float tempBinaryToFahrenheit(uint16_t tempBin);

// I/O unit tasks
static void readSensors(void *pvParameters);
static void sendData(void *pvParameters);

// I/O unit functions
uint16_t readTemp(void);

// ISRs
//void __attribute__( (interrupt(ipl2),
//					 vector(_CHANGE_NOTICE_VECTOR))) CN_ISR_handler( void );
void __attribute__( (interrupt(ipl4),
                     vector(_CAN_1_VECTOR))) CAN1InterruptHandler(void);
void __attribute__( (interrupt(ipl4),
                     vector(_CAN_2_VECTOR))) CAN2InterruptHandler(void);

// hardware setup
static void prvSetupHardware(void);
void PMP_init(void);
void CAN1Init(void);



/* TASK SIGNALS ============================================================= */

xSemaphoreHandle BTN1Pressed;
xSemaphoreHandle BTN2Pressed;
xSemaphoreHandle BTN3Pressed;

xSemaphoreHandle CAN1MsgRcvd;
xSemaphoreHandle CAN2MsgRcvd;



/* GLOBAL VARIABLES ========================================================= */

QueueHandle_t modeQ;
QueueHandle_t tempCurrQ;
QueueHandle_t tempLoQ;
QueueHandle_t tempHiQ;
QueueHandle_t pwmQ;
QueueHandle_t rpsQ;



/* MAIN ===================================================================== */

int main(void)
{
	prvSetupHardware(); /*  Configure hardware */

	LCD_init();
    
	BTN1Pressed = xSemaphoreCreateBinary();
	BTN2Pressed = xSemaphoreCreateBinary();
	BTN3Pressed = xSemaphoreCreateBinary();
    
    CAN1MsgRcvd = xSemaphoreCreateBinary();
    CAN2MsgRcvd = xSemaphoreCreateBinary();
    
    modeQ     = xQueueCreate(1, sizeof(uint8_t));
    tempCurrQ = xQueueCreate(1, sizeof(float));
    tempLoQ   = xQueueCreate(1, sizeof(float));
    tempHiQ   = xQueueCreate(1, sizeof(float));
    pwmQ      = xQueueCreate(1, sizeof(uint8_t));
    rpsQ      = xQueueCreate(1, sizeof(float));
    
	uint8_t mode    = CONFIG;
    float temp_curr = 0;
    float temp_lo   = -1000;
    float temp_hi   = 1000;
    uint8_t pwm     = 0;
    float rps       = 0;
    
    LATGCLR = LED1;
    
    xQueueSend(modeQ, &mode, 0);
    xQueueSend(tempCurrQ, &temp_curr, 0);
    xQueueSend(tempLoQ, &temp_lo, 0);
    xQueueSend(tempHiQ, &temp_hi, 0);
    xQueueSend(pwmQ, &pwm, 0);
    xQueueSend(rpsQ, &rps, 0);
    
    poll_btn_args_t btn1_struct;
	poll_btn_args_t btn2_struct;
	poll_btn_args_t btn3_struct;

	btn1_struct.btn = BTN1;
	btn1_struct.port = IOPORT_G;
	btn1_struct.unblock = BTN1Pressed;

	btn2_struct.btn = BTN2;
	btn2_struct.port = IOPORT_G;
	btn2_struct.unblock = BTN2Pressed;

	btn3_struct.btn = BTN3;
	btn3_struct.port = IOPORT_A;
	btn3_struct.unblock = BTN3Pressed;
    
    set_temp_args_t lo_struct;
    set_temp_args_t hi_struct;
    
    lo_struct.queue = tempLoQ;
    lo_struct.unblock = BTN3Pressed;
    
    hi_struct.queue = tempHiQ;
    hi_struct.unblock = BTN2Pressed;

	if ((modeQ != NULL) && (tempCurrQ != NULL) && (tempLoQ != NULL)
        && (tempHiQ != NULL) && (pwmQ != NULL) && (rpsQ != NULL)
        && (BTN1Pressed != NULL) && (BTN2Pressed != NULL)
        && (BTN3Pressed != NULL))
	{
		// create tasks and start scheduler
		xTaskCreate(pollBTN, "BTN1 Poll", configMINIMAL_STACK_SIZE, &btn1_struct, 2, NULL);
		xTaskCreate(pollBTN, "BTN2 Poll", configMINIMAL_STACK_SIZE, &btn2_struct, 2, NULL);
		xTaskCreate(pollBTN, "BTN3 Poll", configMINIMAL_STACK_SIZE, &btn3_struct, 2, NULL);
		xTaskCreate(printToLCD, "Print to LCD", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		xTaskCreate(switchMode, "Mode Switch", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		xTaskCreate(setTemp, "Low Temp Set", configMINIMAL_STACK_SIZE, &lo_struct, 1, NULL);
		xTaskCreate(setTemp, "High Temp Set", configMINIMAL_STACK_SIZE, &hi_struct, 1, NULL);
        
        xTaskCreate(readSensors, "Sensor Read", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

		vTaskStartScheduler();
	}

	// spin if there isn't enough memory for the heap or if message buffer
	// creation fails
	while (1);

	return 1;
}  /* end of main ----------------------------------------------------------- */



/* CONTROL UNIT TASK DEFINITIONS ============================================ */

static void pollBTN(void *pvParameters)
{
	poll_btn_args_t* params = (poll_btn_args_t*)pvParameters;
    
    const unsigned int btn = params->btn;
    const unsigned int port = params->port;
    xSemaphoreHandle unblock = params->unblock;

	unsigned int btn_curr = 0;
	unsigned int btn_prev = 0;

	const TickType_t period = 1 / portTICK_RATE_MS;
	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1)
	{
		btn_curr = PORTReadBits(port, btn);

		if (btn_curr && !btn_prev) // if button has been pressed
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
}  /* end of pollBTN -------------------------------------------------------- */

static void printToLCD(void* pvParameters)
{
    uint8_t mode;
    float temp_curr;
    float temp_lo;
    float temp_hi;
    uint8_t pwm;
    float rps;
    
	char temp_curr_str[5];
	char temp_lo_str[5];
	char temp_hi_str[5];
	char pwm_str[4];
	char rps_str[6];

	while (1)
	{
        xQueuePeek(modeQ, &mode, 0);
        
        xQueuePeek(tempCurrQ, &temp_curr, 0);
        xQueuePeek(tempLoQ, &temp_lo, 0);
        xQueuePeek(tempHiQ, &temp_hi, 0);
                
		sprintf(temp_curr_str, "%2.1f", temp_curr);
        
        if (temp_lo > -1000)
        {
            sprintf(temp_lo_str, "%2.1f", temp_lo);
        }
        else
        {
            sprintf(temp_lo_str, "");
        }
        
        if (temp_hi < 1000)
        {
            sprintf(temp_hi_str, "%2.1f", temp_hi);
        }
        else
        {
            sprintf(temp_hi_str, "");
        }

		if (mode == OPERATE)
		{
            xQueuePeek(pwmQ, &pwm, 0);
            xQueuePeek(rpsQ, &rps, 0);
            
            sprintf(pwm_str, "%d%%", pwm);
            sprintf(rps_str, "%3.2f", rps);
            
			LCD_clear();
            vTaskDelay(1);

			LCD_set_cursor_pos(1, 6);
			LCD_puts(temp_curr_str);
            vTaskDelay(1);

			LCD_set_cursor_pos(1, 0);
			LCD_puts(temp_lo_str);
            vTaskDelay(1);

			LCD_set_cursor_pos(1, 12);
			LCD_puts(temp_hi_str);
            vTaskDelay(1);

			LCD_set_cursor_pos(0, 0);
			LCD_puts(pwm_str);
            vTaskDelay(1);

			LCD_set_cursor_pos(0, 10);
			LCD_puts(rps_str);
		}
		else  // if config mode
		{
			LCD_clear();
            vTaskDelay(1);

			LCD_set_cursor_pos(0, 6);
			LCD_puts(temp_curr_str);
            vTaskDelay(1);

			LCD_set_cursor_pos(1, 0);
			LCD_puts(temp_lo_str);
            vTaskDelay(1);

			LCD_set_cursor_pos(1, 12);
			LCD_puts(temp_hi_str);
		}
        
        vTaskDelay(100 / portTICK_RATE_MS);
	}
}  /* end of printToLCD ----------------------------------------------------- */

static void switchMode(void *pvParameters)
{
    uint8_t mode;
    uint8_t mode_next;
    
    float temp_lo;
    float temp_hi;
    
	xSemaphoreTake(BTN1Pressed, 0);

	while (1)
	{
		xSemaphoreTake(BTN1Pressed, portMAX_DELAY);
        
        xQueuePeek(modeQ, &mode, 0);
        xQueuePeek(tempLoQ, &temp_lo, 0);
        xQueuePeek(tempHiQ, &temp_hi, 0);

		// if operational mode or config mode and temps set correctly
		if ((mode == OPERATE) || ((mode == CONFIG) && (temp_lo > -1000) && (temp_hi < 1000)))
		{
			if (mode == OPERATE)
            {
                mode_next = CONFIG;
            }
            else
            {
                mode_next = OPERATE;
            }
            
            xQueueOverwrite(modeQ, &mode_next);

			LATGINV = LED1;
		}
	}
}  /* end of switchMode ----------------------------------------------------- */

static void setTemp(void *pvParameters)
{
    set_temp_args_t* params = (set_temp_args_t*)pvParameters;
    QueueHandle_t queue = params->queue;
    xSemaphoreHandle unblock = params->unblock;
    
    float temp_lo;
    float temp_hi;    
    float temp_new = 89;
    
	xSemaphoreTake(unblock, 0);

	while (1)
	{
		xSemaphoreTake(unblock, portMAX_DELAY);

        xQueuePeek(tempCurrQ, &temp_new, 0);
        xQueueOverwrite(queue, &temp_new);
        
        xQueuePeek(tempLoQ, &temp_lo, 0);
        xQueuePeek(tempHiQ, &temp_hi, 0);
        
        if (temp_lo >= temp_hi)
        {
            temp_new = -1000;
            xQueueOverwrite(tempLoQ, &temp_new);
            
            temp_new = 1000;
            xQueueOverwrite(tempHiQ, &temp_new);
        }
	}
}  /* end of setTemp -------------------------------------------------------- */

static void sendPWM(void *pvParameters)
{
    // take semaphore 
    
	while (1)
	{
        // take semaphore
		// construct data frame
		// send data frame

		LATBINV = LEDC;
	}
}  /* end of sendPWM -------------------------------------------------------- */

static void requestData(void* pvParameters)
{
	while (1)
	{
		// construct RTR frame
		// send RTR frame

		LATBINV = LEDA;
        
        vTaskDelay(2000 / portTICK_RATE_MS);
	}
}  /* end of requestData ---------------------------------------------------- */

static void CAN1RXHandler(void* pvParameters)
{
    xSemaphoreTake(CAN1MsgRcvd, 0);
    
    while (1)
    {
        xSemaphoreTake(CAN1MsgRcvd, portMAX_DELAY);
    }
}  /* end of CAN1RXHandler -------------------------------------------------- */



/* CONTROL UNIT FUNCTION DEFINITIONS ======================================== */

float tempBinaryToFahrenheit(uint16_t tempBin)
{
    float kelvin;
    float celsius;
    float fahrenheit;
    
    kelvin = tempBin * 0.02f;
    celsius = kelvin - 273.15f;
    fahrenheit = (celsius * (9.0f/5.0f)) + 32;
    
    return fahrenheit;
}



/* I/O UNIT TASK DEFINITIONS ================================================ */

static void readSensors(void* pvParameters)
{
    uint16_t temp;
    float fahrenheit;
    
	while (1)
	{
		// read
        temp = readTemp();
        fahrenheit = tempBinaryToFahrenheit(temp);
        xQueueOverwrite(tempCurrQ, &fahrenheit);
        
        vTaskDelay(500 / portTICK_RATE_MS);
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



/* I/O UNIT FUNCTION DEFINITIONS ============================================ */

uint16_t readTemp(void)
{
    uint16_t temp;
    
    // SMBus parameters
    const uint8_t slave_addr = 0x5a;
    const uint8_t command = 0x07;
    uint8_t ir_data[3];
    const int data_len = 3;
    
    I2C1_IR_Read(slave_addr, command, ir_data, data_len);
    
    temp = (ir_data[1] << 8) | ir_data[0];
    
    return temp;
}



/* INTERRUPT SERVICE ROUTINES =============================================== */

/****************************************************************************
 * Function:    void __ISR(_CAN_1_VECTOR, ipl4) CAN1InterruptHandler(void);
 * Description:
 *  This is the CAN1 Interrupt Handler. Note that there are many source events
 *  in the CAN1 module for this interrupt. These events are enabled by the
 *  CANEnableModuleEvent() function. In this example, only the RX_EVENT
 *  is enabled.
 * Precondition:    None.
 * Parameters:      None.
 * Return Values:   None.
 * Remarks:         ISR cannot be called directly.
  ***************************************************************************/
void CAN1InterruptHandler(void)
{

/* Check if the source of the interrupt is  RX_EVENT. This is redundant since
 * only this event is enabled in this example but this shows one scheme for
 * handling events. */

    if((CANGetModuleEvent(CAN1) & CAN_RX_EVENT) != 0)
    {
/* Within this, you can check which channel caused the event by using the 
 * CANGetModuleEvent() function which returns a code representing the highest
 * priority pending event. */
        if(CANGetPendingEventCode(CAN1) == CAN_CHANNEL1_EVENT)
        {
/* This means that channel 1 caused the event. The CAN_RX_CHANNEL_NOT_EMPTY
 * event is persistent. You could either read the channel in the ISR
 * to clear the event condition or as done here, disable the event source,
 * and set an application flag to indicate that a message  has been received.
 * The event can be enabled by the application when it has processed one
 * message. */

/* Note that leaving the event enabled would cause the CPU to keep executing
 * the ISR since the CAN_RX_CHANNEL_NOT_EMPTY event is persistent (unless
 * the not empty condition is cleared.)  */
            CANEnableChannelEvent(CAN1, CAN_CHANNEL1, CAN_RX_CHANNEL_NOT_EMPTY,
                                    FALSE);

            xSemaphoreGive(CAN1MsgRcvd);  // PJP
        }
    }

/* The CAN1 Interrupt flag is cleared at the end of the interrupt routine. 
 * This is because the interrupt source that could have caused this interrupt
 * to occur (CAN_RX_CHANNEL_NOT_EMPTY) is disabled. Attempting to clear the
 * CAN1 interrupt flag when the the CAN_RX_CHANNEL_NOT_EMPTY interrupt is
 * enabled will not have any effect because the base event is still present. */	
    INTClearFlag(INT_CAN1);
}

/****************************************************************************
 * Function:    void __ISR(_CAN_2_VECTOR, ipl4) CAN2InterruptHandler(void);
 * Description:
 * This is the CAN2 Interrupt Handler. Note that there are many events in the
 * CAN2 module that can cause this interrupt. These events are enabled by the
 * CANEnableModuleEvent() function. In this example, only the RX_EVENT is
 * enabled.
 * Precondition:    None.
 * Parameters:      None.
 * Return Values:   None.
 * Remarks:         ISR cannot be called directly
  ***************************************************************************/
void CAN2InterruptHandler(void)
{  
/* Check if the source of the interrupt is RX_EVENT. This is redundant since 
 * only this event is enabled in this example but this shows one scheme for
 * handling interrupts. */

    if((CANGetModuleEvent(CAN2) & CAN_RX_EVENT) != 0)
    {
/* Within this, you can check which event caused the interrupt by using the 
 * CANGetPendingEventCode() function to get a code representing the highest
 * priority active event.*/
		
        if(CANGetPendingEventCode(CAN2) == CAN_CHANNEL1_EVENT)
        {
/* This means that channel 1 caused the event. The CAN_RX_CHANNEL_NOT_EMPTY
 * event is persistent. You could either read the channel in the ISR
 * to clear the event condition or as done here, disable the event source,
 * and set  an application flag to indicate that a message has been received.
 * The event can be enabled by the application when it has processed one
 * message.
 *
 * Note that leaving the event enabled would cause the CPU to keep executing the
 * ISR since the CAN_RX_CHANNEL_NOT_EMPTY event is persistent (unless the not
 * empty condition is cleared.)
 */			
            CANEnableChannelEvent(CAN2, CAN_CHANNEL1, CAN_RX_CHANNEL_NOT_EMPTY, 
                                    FALSE);

            xSemaphoreGive(CAN2MsgRcvd);  // PJP
        }
    }

/* The CAN2 Interrupt flag is  cleared at the end of the interrupt routine. 
 * This is because the event that could have caused this interrupt  to occur
 * (CAN_RX_CHANNEL_NOT_EMPTY) is disabled. Attempting to clear the CAN2
 * interrupt flag when the the CAN_RX_CHANNEL_NOT_EMPTY interrupt is enabled
 * will not have any effect because the base event is still present. */
	
    INTClearFlag(INT_CAN2);
} /* enD OF CAN2InterruptHandler(void) */



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

/* CAN1Init Function Description ********************************************
SYNTAX:         void CAN1Init(void);
KEYWORDS:       CAN1, initialize
DESCRIPTION:    This function initializes CAN1 for extended 29 bit ID
PARAMETER1:     None
RETURN VALUE:   None
Notes:          None
END DESCRIPTION ************************************************************/
void CAN1Init(void)
{
CAN_BIT_CONFIG canBitConfig;

/* chipKIT Pro MX7 +++++++++++++++++++++ */
    PORTSetPinsDigitalIn(IOPORT_F, BIT_12);	/* Set CAN1 Rx */
    PORTSetPinsDigitalOut(IOPORT_F, BIT_13);    /* Set CAN1 Tx */
    ODCFSET = BIT_13;        /* Set CAN Tx IO Pin for open drain */
/*++++++++++++++++++++++++++++++++++++++ */

/* This function will initialize CAN1 module. */
/* Step 1: Switch the CAN module ON and switch it to Configuration
 *  mode. Wait till the mode switch is complete. */

    CANEnableModule(CAN1,TRUE);

/* CAN set Operation Mode parameter description:
 * This routine sets the CAN operating mode. The CAN module requires itself to
 * be in certain modes in order to gain access to module functionality. Note
 * that after this function is called, it should be checked whether the mode
 * was set by using the CANGetOperatingMode() function.
 *
 * Parameter 1:     CAN Module:  CAN1 or CAN2
 * Parameter 2:     CAN Operation Mode - mutually exclusive:
 *                      CAN_NORMAL_OPERATION,
 *                      CAN_DISABLE
 *                      CAN_LOOPBACK
 *                      CAN_LISTEN_ONLY
 *                      CAN_CONFIGURATION
 *                      CAN_LISTEN_ALL_MESSAGES
*/
    CANSetOperatingMode(CAN1, CAN_CONFIGURATION);
    while(CANGetOperatingMode(CAN1) != CAN_CONFIGURATION);

 /* Step 2: Configure the Clock.The CAN_BIT_CONFIG data structure is used
  * for this purpose. The propagation segment, phase segment 1 and phase
  * segment 2 are configured to have 3TQ. CAN_BUS_SPEED is defined
  * in CANFunctions.h. GetSystemClock() is defined in chipKIT_Pro_MX7.h  */
	
    canBitConfig.phaseSeg2Tq            = CAN_BIT_3TQ;
    canBitConfig.phaseSeg1Tq            = CAN_BIT_3TQ;
    canBitConfig.propagationSegTq       = CAN_BIT_3TQ;
    canBitConfig.phaseSeg2TimeSelect    = TRUE;
    canBitConfig.sample3Time            = TRUE;
    canBitConfig.syncJumpWidth          = CAN_BIT_2TQ;

    CANSetSpeed(CAN1,&canBitConfig, GetSystemClock(), CAN_BUS_SPEED);

/* Step 3: Assign the buffer area to the CAN module. */

/* CAN Assign Channel Memory parameter description:
 * This routine assigns buffer memory to the CAN module. The CAN module uses
 * this buffer memory to store messages to be transmitted and received. The
 * size of the memory buffer should be enough to accommodate the required
 * number of message buffers and channels.
 *
 * Parameter 1:     CAN Module:  CAN1 or CAN2
 * Parameter 2:     CAN FIFO Memory - pointer to RAM allocated for channel msgs.
 * Parameter 4:     CAN FIFO Memory size - total number of bytes for RX and TX
*/
    CANAssignMemoryBuffer(CAN1,CAN1MessageFifoArea,(CAN1_MSG_MEMORY));

/* Step 4: Configure channel 0 for TX and size of 8 message buffers with RTR
 * disabled and low medium priority. Configure channel 1 for RX and size
 * of 8 message buffers and receive the full message. */

/* CAN Configure Channel for TX parameter description:
 * Parameter 1:     CAN Module:  CAN1 or CAN2
 * Parameter 2:     CAN FIFO Channel Number: CAN_CHANNEL0 to CAN_CHANNEL31
 * Parameter 3:     Numbers of FIFO buffers - 1 to 32
 * Parameter 4:     CAN RTR Request: CAN_TX_RTR_DISABLED / CAN_TX_RTR_ENABLED
 * Parameter 5:     CAN Sending priority:
 *                      CAN_LOWEST_PRIORITY
 *                      CAN_LOW_MEDIUM_PRIORITY
 *                      CAN_HIGH_MEDIUM_PRIORITY
 *                      CAN_HIGHEST_PRIORITY
*/
    CANConfigureChannelForTx(CAN1, CAN_CHANNEL0, CAN1_FIFO_BUFFERS,
                             CAN_TX_RTR_DISABLED,
                             CAN_LOW_MEDIUM_PRIORITY);


/* CAN Configure Channel for RX parameter description:
 * Parameter 1:     CAN Module:  CAN1 or CAN2
 * Parameter 2:     CAN FIFO Channel Number: CAN_CHANNEL0 to CAN_CHANNEL31
 * Parameter 3:     Numbers of FIFO buffers - 1 to 32
 * Parameter 4:     CAN RTR RX mode: CAN_RX_DATA_ONLY or CAN_RX_FULL_RECEIVE
*/
    CANConfigureChannelForRx(CAN1, CAN_CHANNEL1, CAN1_FIFO_BUFFERS,
                             CAN_RX_FULL_RECEIVE);
	
/* Step 5: Configure filters and mask. Configure filter 0 to accept EID 
 * messages with ID 0x8004001. Configure filter mask 0 to compare all the ID
 * bits and to filter by the ID type specified in the filter configuration.
 * Messages accepted by filter 0 should be stored in channel 1. */

/* CAN Configure Filter parameter description:
 * Parameter 1:     CAN Module:  CAN1 or CAN2
 * Parameter 2:     CAN Filter number: CAN_FILTER0 - CAN_FILTER31
 * Parameter 3:     CAN ID: 0x000000000 to 0x01FFFFFFF
 * Parameter 4:     CAN ID Type: CAN_EID or CAN_SID
 */
    CANConfigureFilter(CAN1, CAN_FILTER0, CAN_EID_MSG_1, CAN_EID);

/* CAN Configure Filter Mask parameter description:
 * Parameter 1:     CAN Module:  CAN1 or CAN2
 * Parameter 2:     CAN Filter Mask: CAN_FILTER_MASK0 - CAN_FILTER_MASK3
 * Parameter 3:     CAN Mask:   0x0 - 0x01FFFFFFF
 * Parameter 4:     CAN ID Type: CAN_EID or CAN_SID
 * Parameter 5:     CAN Filter Mask Type: CAN_FILTER_MASK_IDE_TYPE or
 *                                        CAN_FILTER_MASK_ANY_TYPE
 */
    CANConfigureFilterMask(CAN1, CAN_FILTER_MASK0, EID_FILTER_MASK,
                              CAN_EID, CAN_FILTER_MASK_IDE_TYPE);

/* CAN Link Channel to Filter to Mask parameter description:
 * Parameter 1:     CAN Module:  CAN1 or CAN2
 * Parameter 2:     CAN Filter number: CAN_FILTER0 to CAN_FILTER31
 * Parameter 3:     CAN Mask number: CAN_FILTER_MASK0 to CAN_FILTER_MASK3
 * Parameter 4:     CAN FIFO Channel Number: CAN_CHANNEL0 to CAN_CHANNEL31
 */
    CANLinkFilterToChannel(CAN1, CAN_FILTER0, CAN_FILTER_MASK0, CAN_CHANNEL1);

/* CAN Filter Enable parameter description:
 * Parameter 1:     CAN Module:  CAN1 or CAN2
 * Parameter 2:     CAN Filter number: CAN_FILTER0 to CAN_FILTER31
 * Parameter 3:     ENABLE: TURE or FALSE
 */
    CANEnableFilter(CAN1, CAN_FILTER0, TRUE);
	
/* Step 6: Enable interrupt and events. Enable the receive channel not empty
 * event (channel event) and the receive channel event (module event).
 * The interrupt peripheral library is used to enable the CAN interrupt to
 * the CPU. */

/* CAN Enable Channel Event parameter description:
 * This routine enables or disables channel level events. Any enabled channel
 * event will cause a CAN module event. An event can be active regardless of
 * it being enabled or disabled. Enabling a TX type of event for a RX channel
 * will have no effect.
 * 
 * Parameter 1:     CAN Module:  CAN1 or CAN2
 * Parameter 2:     CAN FIFO Channel Number: CAN_CHANNEL0 to CAN_CHANNEL31
 * Parameter 3:     CAN Channel event type - any combination:   	
 *                      CAN_RX_CHANNEL_NOT_EMPTY
 *                      CAN_RX_CHANNEL_HALF_FULL
 *                      CAN_RX_CHANNEL_FULL
 *                      CAN_RX_CHANNEL_OVERFLOW
 *                      CAN_RX_CHANNEL_ANY_EVENT
 *                      CAN_TX_CHANNEL_EMPTY
 *                      CAN_TX_CHANNEL_HALF_EMPTY
 *                      CAN_TX_CHANNEL_NOT_FULL
 *                      CAN_TX_CHANNEL_ANY_EVENT
 *  Parameter 4:    Event enable - TRUE or FALSE 
 */
    CANEnableChannelEvent(CAN1, CAN_CHANNEL1, CAN_RX_CHANNEL_NOT_EMPTY, TRUE);

/* CAN Enable Module Event parameter description:
 * This routine enables or disables module level events. Any enabled module
 * event will cause the CAN module to generate a CPU interrupt. An event can
 * be active regardless of it being enabled or disabled.
 *
 * Parameter 1:     CAN Module:  CAN1 or CAN2
 * Parameter 2:     CAN Event flags - any combination:
 *                      CAN_TX_EVENT
 *                      CAN_RX_EVENT
 *                      CAN_TIMESTAMP_TIMER_OVERFLOW_EVENT
 *                      CAN_OPERATION_MODE_CHANGE_EVENT
 *                      CAN_RX_OVERFLOW_EVENT
 *                      CAN_SYSTEM_ERROR_EVENT
 *                      CAN_BUS_ERROR_EVENT
 *                      CAN_BUS_ACTIVITY_WAKEUP_EVENT
 *                      CAN_INVALID_RX_MESSAGE_EVENT
 * Parameter 3:     Enable CAN events: TRUE or FALSE
*/
    CANEnableModuleEvent (CAN1, CAN_RX_EVENT, TRUE);

/* These functions are from interrupt peripheral library. */
    INTSetVectorPriority(INT_CAN_1_VECTOR, INT_PRIORITY_LEVEL_4);
    INTSetVectorSubPriority(INT_CAN_1_VECTOR, INT_SUB_PRIORITY_LEVEL_0);
    INTEnable(INT_CAN1, INT_ENABLED);

/* Step 7: Switch the CAN mode to normal mode. */
    CANSetOperatingMode(CAN1, CAN_NORMAL_OPERATION);
    while(CANGetOperatingMode(CAN1) != CAN_NORMAL_OPERATION);

} /* End of CAN1Init */

/* CAN2Init Function Description ********************************************
SYNTAX:         void CAN2Init(void);
KEYWORDS:       CAN2, initialize
DESCRIPTION:    This function initializes CAN2 for extended 29 bit ID
PARAMETER1:     None
RETURN VALUE:   None
Notes:          See CAN1Init for function descriptions and program details
END DESCRIPTION ************************************************************/
void CAN2Init(void)
{
    CAN_BIT_CONFIG canBitConfig;

/* chipKIT Pro MX7 +++++++++++++++++++++ */
    PORTSetPinsDigitalIn(IOPORT_C, BIT_3);  /* Set CAN2 Rx */
    PORTSetPinsDigitalOut(IOPORT_C, BIT_2); /* Set CAN2 Tx */
    ODCCSET = BIT_2;         /* Set CAN Tx IO pin for open drain */
/* +++++++++++++++++++++++++++++++++++++ */

 /* Step 1: Enable CAN Module and set into configuration mode. */
    CANEnableModule(CAN2,TRUE);

    CANSetOperatingMode(CAN2, CAN_CONFIGURATION);
    while(CANGetOperatingMode(CAN2) != CAN_CONFIGURATION);

 /* Step 2: Configure the Clock. */
	
    canBitConfig.phaseSeg2Tq            = CAN_BIT_3TQ;
    canBitConfig.phaseSeg1Tq            = CAN_BIT_3TQ;
    canBitConfig.propagationSegTq       = CAN_BIT_3TQ;
    canBitConfig.phaseSeg2TimeSelect    = TRUE;
    canBitConfig.sample3Time            = TRUE;
    canBitConfig.syncJumpWidth          = CAN_BIT_2TQ;

    CANSetSpeed(CAN2, &canBitConfig, GetSystemClock(), CAN_BUS_SPEED);

/* Step 3: Assign the buffer area to the CAN module. */
    CANAssignMemoryBuffer(CAN2,CAN2MessageFifoArea,CAN2_MSG_MEMORY);

/* Step 4: Configure channel 0  */
    CANConfigureChannelForTx(CAN2,CAN_CHANNEL0,CAN2_FIFO_BUFFERS,
                             CAN_TX_RTR_DISABLED,
                             CAN_LOW_MEDIUM_PRIORITY);

    CANConfigureChannelForRx(CAN2,CAN_CHANNEL1,CAN2_FIFO_BUFFERS,
                             CAN_RX_FULL_RECEIVE);
	
 /* Step 5: Configure filters and mask. */
   
    CANConfigureFilter(CAN2, CAN_FILTER0, CAN_EID_MSG_4 , CAN_EID);

    CANConfigureFilterMask(CAN2, CAN_FILTER_MASK0, EID_FILTER_MASK,
                                CAN_EID, CAN_FILTER_MASK_IDE_TYPE);

    CANLinkFilterToChannel(CAN2, CAN_FILTER0, CAN_FILTER_MASK0, CAN_CHANNEL1);

    CANEnableFilter(CAN2, CAN_FILTER0, TRUE);
	
/* Step 6: Enable interrupt and events.  */
    CANEnableChannelEvent(CAN2, CAN_CHANNEL1, CAN_RX_CHANNEL_NOT_EMPTY, TRUE);
    CANEnableModuleEvent(CAN2, CAN_RX_EVENT, TRUE);

 /* These functions are from interrupt peripheral library. */
    INTSetVectorPriority(INT_CAN_2_VECTOR, INT_PRIORITY_LEVEL_4);
    INTSetVectorSubPriority(INT_CAN_2_VECTOR, INT_SUB_PRIORITY_LEVEL_0);
    INTEnable(INT_CAN2, INT_ENABLED);
    
 /* Step 7: Switch the CAN mode to normal mode. */
    CANSetOperatingMode(CAN2, CAN_NORMAL_OPERATION);
    while(CANGetOperatingMode(CAN2) != CAN_NORMAL_OPERATION);
} /* End of CAN2Init */



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
	for (;;);
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

	while (1);
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
	for (;;);
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

	for (;;);
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