/******************************************************************************

 Simple demo of creating one server socket for sending the core timer.
 
 Dr. J
 4 Nov2021
 
 Status:
 
 Reduced stack size for the create "server connection instance" tasks
 to 512 and now it seems to work, regardless of order.
 
*******************************************************************************/

#define sourceAddress           { 129, 101, 222, 24 } // work
#define listeningPort           10000

/* Standard includes. */
#include <plib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Hardware dependent setting */
#include "CerebotMX7cK.h"
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"

#define tcpechoSHUTDOWN_DELAY	( pdMS_TO_TICKS( 5000 ) )

#define BUF_LEN     8

void configDMA(void);
static void dmaFillBufs(void* pvParameters);
void __attribute__( (interrupt(ipl2),
                     vector(_DMA3_VECTOR))) DMAInterruptHandler(void);

unsigned int src_buf[BUF_LEN];
unsigned int dest_buf[BUF_LEN];

void configADC(void);
void __attribute__( (interrupt(ipl1),
                     vector(_ADC_VECTOR))) ADCInterruptHandler(void);

SemaphoreHandle_t sendTCP;

/* Set up the processor for the example program. */
static void prvSetupHardware( void );

/* Task that waits for incoming TCP connections*/
static void vCreateTCPServerSocket( void *pvParameters );
/* Task that sends the core timer via TCP packets*/
static void prvServerConnectionInstance( void *pvParameters );

/* The MAC address array is not declared const as the MAC address will
normally be read from an EEPROM and not hard coded (in real deployed
applications). In this case the MAC Address is hard coded to the value we
have for the Cerebot board we're using. */
static uint8_t ucMACAddress[ 6 ] = { 0x00, 0x04, 0xA3, 0x1A, 0x4D, 0x90 };

/* Set this value to be the IP address you want to use (you are recommended 
 * to use the value given with your Cerebot station but the value doesn't seem
 * to be strictly limited to that).*/
static const uint8_t ucIPAddress[ 4 ] = sourceAddress;

/* You shouldn't need to worry about these values as we're using DHCP. */
static const uint8_t ucNetMask[ 4 ] = { 255, 255, 252, 0 };
static const uint8_t ucGatewayAddress[ 4 ] = { 129, 101, 220, 1 }; // work
/* The following is the address of an OpenDNS server. */
static const uint8_t ucDNSServerAddress[ 4 ] = { 0, 0, 0, 0 };

int main( void )
{
    prvSetupHardware();
    /* Initialize the RTOS's TCP/IP stack. This initializes the MAC and kicks off
     * the network management task "prvIPTask" which will be managing our network
     * events */
    
    FreeRTOS_IPInit( ucIPAddress,
                     ucNetMask,
                     ucGatewayAddress,
                     ucDNSServerAddress,
                     ucMACAddress );
    
    sendTCP = xSemaphoreCreateBinary();

    /*
     * Our RTOS tasks can be created here.
     */
    xTaskCreate( vCreateTCPServerSocket, "TCP1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
//    xTaskCreate(dmaFillBufs, "DMA", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
   
    /* Start the RTOS scheduler. */
    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running. */
    for( ;; );
    
    /* Will only reach here if there is insufficient heap available to start
     * the scheduler. */
    return 0;
}  /* End of main */

/* prvSetupHardware Function Description ***************************************
 * SYNTAX:	static void prvSetupHardware( void );
 * KEYWORDS:	Hardware, initialize, configure, setup
 * DESCRIPTION: Initializes hardware specific resources.
 * PARAMETERS:	None
 * RETURN VALUE: None
 * NOTES:	Static function - can be called exclusively from 
 * 		within this program.
 * END DESCRIPTION ************************************************************/
static void prvSetupHardware( void )
{
    Cerebot_mx7cK_setup();
    
    /* Enable chipKIT Pro MX7 and Cerebot 32MX7ck PHY 
     * (this is essential for using the PHY chip)*/
    TRISACLR = (unsigned int) BIT_6; // Make bit output
    LATASET = (unsigned int) BIT_6;	 // Set output high
    
    PORTSetPinsDigitalOut(IOPORT_G, BRD_LEDS);
    LATGCLR = BRD_LEDS;
    
    configDMA();    
    configADC();
    
    /* Enable multi-vector interrupts */
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);  /* Do only once */
    INTEnableInterrupts();   /* Do as needed for global interrupt control */
    portDISABLE_INTERRUPTS();
} /* End of prvSetupHardware */

void configDMA(void)
{
    DmaChnOpen(3, DMA_CHN_PRI2, DMA_OPEN_DEFAULT);
    DmaChnSetEvEnableFlags(3, DMA_EV_CELL_DONE);
    mDmaChnIntEnable(3);
    mDmaChnSetIntPriority(3, 2, 1);
    
    unsigned int buf_size = BUF_LEN * sizeof(unsigned int);
    DmaChnSetTxfer(3, src_buf, dest_buf, buf_size, buf_size, buf_size);
//    DmaChnDisable(3);
}

void configADC(void)
{
    CloseADC10();
    
    unsigned int config1 = ADC_MODULE_ON | ADC_FORMAT_INTG16 | ADC_CLK_AUTO |
                           ADC_AUTO_SAMPLING_ON;
    unsigned int config2 = ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE |
                           ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_8 |
                           ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF;
    unsigned int config3 = ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_12 |
                           ADC_CONV_CLK_25Tcy;
    unsigned int configport = ENABLE_AN2_ANA;
    unsigned int configscan = SKIP_SCAN_ALL;
    
    SetChanADC10(ADC_CH0_POS_SAMPLEA_AN2 | ADC_CH0_NEG_SAMPLEA_NVREF);
    OpenADC10(config1, config2, config3, configport, configscan);
    ConfigIntADC10(ADC_INT_ON | ADC_INT_PRI_1 | ADC_INT_SUB_PRI_1);
//    CloseADC10();
}

/* vApplicationStackOver Function Description **********************************
 * SYNTAX:          void vApplicationStackOverflowHook( void );
 * KEYWORDS:        Stack, overflow
 * DESCRIPTION:     Look at pxCurrentTCB to see which task overflowed
 *                  its stack.
 * PARAMETERS:      None
 * RETURN VALUE:    None
 * NOTES:           See FreeRTOS documentation
 * END DESCRIPTION ************************************************************/
void vApplicationStackOverflowHook ( TaskHandle_t xTask,
                                               char * pcTaskName )
{
	for( ;; );
} /* End of vApplicationStackOver */

/* _general_exception_handler Function Description *****************************
 * SYNTAX:          void _general_exception_handler( unsigned long ulCause,
 *                                              unsigned long ulStatus );
 * KEYWORDS:        Exception, handler
 * DESCRIPTION:     This overrides the definition provided by the kernel.
 *                  Other exceptions should be handled here. Set a breakpoint
 *                  on the "for( ;; )" to catch problems.
 * PARAMETER 1:     unsigned long - Cause of exception code
 * PARAMETER 2:     unsigned long - status of process
 * RETURN VALUE:    None
 * NOTES:           Program will be vectored to here if the any CPU error is
 *                  generated. See FreeRTOS documentation for error codes.
END DESCRIPTION ***************************************************************/
void _general_exception_handler( unsigned long ulCause, unsigned long ulStatus )
{
    for( ;; );
} /* End of _general_exception_handler */

static void dmaFillBufs(void* pvParameters)
{    
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        PORTGINV = LED1;
    }
}

void DMAInterruptHandler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    int flags = DmaChnGetEvFlags(3);
    
    if (flags & DMA_EV_CELL_DONE)
    {
        LATBINV = LEDD;
        xSemaphoreGiveFromISR(sendTCP, &xHigherPriorityTaskWoken);
    }
    
    DmaChnClrEvFlags(3, DMA_EV_ALL_EVNTS);
    DmaChnClrIntFlag(3);
}

void ADCInterruptHandler(void)
{
    unsigned int buf0;
    unsigned int active_buf = ReadActiveBufferADC10();
    
    if (active_buf)
    {
        src_buf[0] = ADC1BUF8;
        src_buf[1] = ADC1BUF9;
        src_buf[2] = ADC1BUFA;
        src_buf[3] = ADC1BUFB;
        src_buf[4] = ADC1BUFC;
        src_buf[5] = ADC1BUFD;
        src_buf[6] = ADC1BUFE;
        src_buf[7] = ADC1BUFF;
    }
    else
    {
        src_buf[0] = ADC1BUF0;
        src_buf[1] = ADC1BUF1;
        src_buf[2] = ADC1BUF2;
        src_buf[3] = ADC1BUF3;
        src_buf[4] = ADC1BUF4;
        src_buf[5] = ADC1BUF5;
        src_buf[6] = ADC1BUF6;
        src_buf[7] = ADC1BUF7;
    }
    
    buf0 = src_buf[0];
    PORTGINV = LED2;
    DmaChnStartTxfer(3, DMA_WAIT_NOT, 0);
    mAD1ClearIntFlag();
}

/* vCreateTCPServerSocket Function Description *************************
 * SYNTAX:          static void vCreateTCPServerSocket( void *pvParameters );
 * KEYWORDS:        RTOS, Task
 * DESCRIPTION:     Waits for an incoming request for a TCP socket connection 
 *                  to be made. The function then, launches a new task for 
 *                  managing the connection and deletes itself.
 * PARAMETER 1:     void pointer - data of unspecified data type sent from
 *                  RTOS scheduler
 * RETURN VALUE:    None (There is no returning from this function)
END DESCRIPTION ************************************************************/
static void vCreateTCPServerSocket( void *pvParameters )
{
    struct freertos_sockaddr xClient, xBindAddress;
    Socket_t xListeningSocket, xConnectedSocket;
    socklen_t xSize = sizeof( xClient );
    static const TickType_t xReceiveTimeOut = portMAX_DELAY;
    const BaseType_t xBacklog = 20;
    BaseType_t xReturned;

    /* Attempt to open the socket. */
    xListeningSocket = FreeRTOS_socket( FREERTOS_AF_INET,
					FREERTOS_SOCK_STREAM,/* FREERTOS_SOCK_STREAM for TCP. */
					FREERTOS_IPPROTO_TCP );

    /* Check the socket was created. */
    configASSERT( xListeningSocket != FREERTOS_INVALID_SOCKET );

    /* Set a time out so accept() will just wait for a connection. */
    FreeRTOS_setsockopt( xListeningSocket,
                         0,
                         FREERTOS_SO_RCVTIMEO,
                         &xReceiveTimeOut,
                         sizeof( xReceiveTimeOut ) );

    /* Set the listening port. */
    xBindAddress.sin_port = ( uint16_t ) listeningPort;
    xBindAddress.sin_port = FreeRTOS_htons( xBindAddress.sin_port );

    /* Bind the socket to the port that the client RTOS task will send to. */
    FreeRTOS_bind( xListeningSocket, &xBindAddress, sizeof( xBindAddress ) );

    /* Set the socket into a listening state so it can accept connections.
    The maximum number of simultaneous connections is limited to 20. */
    FreeRTOS_listen( xListeningSocket, xBacklog );

    for( ;; )
    {
        /* Wait for incoming connections. */
        xConnectedSocket = FreeRTOS_accept( xListeningSocket, &xClient, &xSize );
        configASSERT( xConnectedSocket != FREERTOS_INVALID_SOCKET );

        /* Spawn a RTOS task to handle the connection. */
        xReturned = xTaskCreate( prvServerConnectionInstance,
                     "TimerRpt",
                     512, /* I've increased the memory allocated to the task as I was encountering stack overflow issues */
                     ( void * ) xConnectedSocket,
                     tskIDLE_PRIORITY,
                     NULL );
        
        if (xReturned == pdPASS)
            vTaskDelete( NULL );
        
        while(1);
    } // for(;;)
} // vCreateTCPServerSocket


/* prvServerConnectionInstance Function Description *************************
 * SYNTAX:          static void prvServerConnectionInstance( void *pvParameters );
 * KEYWORDS:        RTOS, Task
 * DESCRIPTION:     Periodically sends the core timer
 * PARAMETER 1:     void pointer - data of unspecified data type sent from
 *                  RTOS scheduler
 * RETURN VALUE:    None (There is no returning from this function)
END DESCRIPTION ************************************************************/
static void prvServerConnectionInstance( void *pvParameters )
{
	int32_t lBytes, lSent, lTotalSent;
	uint8_t cReceivedString[ ipconfigTCP_MSS ];
	Socket_t xConnectedSocket;
	static const TickType_t xReceiveTimeOut = pdMS_TO_TICKS( 5000 );
	static const TickType_t xSendTimeOut = pdMS_TO_TICKS( 5000 );
	TickType_t xTimeOnShutdown;
    
    int i;
//    unsigned int dest_buf[BUF_LEN] = {0, 1, 2, 3, 4, 5, 6, 7};

	xConnectedSocket = ( Socket_t ) pvParameters;
	FreeRTOS_setsockopt( xConnectedSocket, 0, FREERTOS_SO_RCVTIMEO, &xReceiveTimeOut, sizeof( xReceiveTimeOut ) );
	FreeRTOS_setsockopt( xConnectedSocket, 0, FREERTOS_SO_SNDTIMEO, &xSendTimeOut, sizeof( xReceiveTimeOut ) );
    
    xSemaphoreTake(sendTCP, 0);
    
//    DmaChnEnable(3);
//    ConfigIntADC10(ADC_INT_ON);
//    EnableADC10();

	for( ;; )
	{
		xSemaphoreTake(sendTCP, portMAX_DELAY);
        
        sprintf(cReceivedString, "%u, ", dest_buf[0]);
        
        for (i = 1; i < BUF_LEN; i++)
        {
            sprintf(cReceivedString+strlen(cReceivedString), "%u, ", dest_buf[i]);
        }
        
        sprintf(cReceivedString+strlen(cReceivedString), "\r\n", dest_buf[i]);
        
        lBytes = strlen(cReceivedString) + 1;

		/* If Send the string. */
		if( lBytes >= 0 )
		{
		    lSent = 0;
		    lTotalSent = 0;
			
			while( ( lSent >= 0 ) && ( lTotalSent < lBytes ) )
			{
				lSent = FreeRTOS_send( xConnectedSocket, &cReceivedString[lTotalSent], lBytes - lTotalSent, 0 );
				lTotalSent += lSent;
			}

			if( lSent < 0 )
			{
				/* Socket closed? */
				break;
			}
		} // if
		else // lBytes < 0
		{
			/* Socket closed? */
			break;
		}
	} // for(;;)
	
	/* Initiate a shutdown in case it has not already been initiated. */
	FreeRTOS_shutdown( xConnectedSocket, FREERTOS_SHUT_RDWR );

	/* Wait for the shutdown to take effect, indicated by FreeRTOS_recv()
	 * returning an error. */
	xTimeOnShutdown = xTaskGetTickCount();
	do
	{
	    if( FreeRTOS_recv( xConnectedSocket, cReceivedString, ipconfigTCP_MSS, 0 ) < 0 )
	    {
		    break;
	    }
	} while( ( xTaskGetTickCount() - xTimeOnShutdown ) < tcpechoSHUTDOWN_DELAY );

	/* Finished with the socket and the task. */
	FreeRTOS_closesocket( xConnectedSocket );
	vTaskDelete( NULL );
} // prvServerConnectionInstance

/* ulApplicationGetNextSequenceNumber Function Description *********************
 * SYNTAX:          uint32_t ulApplicationGetNextSequenceNumber
 *                                              ( uint32_t ulSourceAddress,
 *												uint16_t usSourcePort,
 *												uint32_t ulDestinationAddress,
 *												uint16_t usDestinationPort );
 * DESCRIPTION:     Callback that provides the inputs necessary to generate a 
 *                  randomized TCP Initial Sequence Number per RFC 6528.  THIS 
 *                  IS ONLY A DUMMY IMPLEMENTATION THAT RETURNS A PSEUDO RANDOM 
 *                  NUMBER SO IS NOT INTENDED FOR USE IN PRODUCTION SYSTEMS.
 * PARAMETER 1:     uint32_t - IP source address
 * PARAMETER 2:     uint32_t - IP source address port
 * PARAMETER 3:     uint32_t - IP destination address
 * PARAMETER 4:     uint32_t - IP destination address port
 * RETURN VALUE:    A randomized TCP Initial Sequence Number.
END DESCRIPTION ************************************************************/
extern uint32_t ulApplicationGetNextSequenceNumber( uint32_t ulSourceAddress,
													uint16_t usSourcePort,
													uint32_t ulDestinationAddress,
													uint16_t usDestinationPort )
{
	( void ) ulSourceAddress;
	( void ) usSourcePort;
	( void ) ulDestinationAddress;
	( void ) usDestinationPort;

	return uxRand();
}

/* uxRand Function Description *********************************************
 * SYNTAX:          UBaseType_t uxRand( void );
 * DESCRIPTION:     Function called by the IP stack to generate random numbers for
 *                  things such as a DHCP transaction number or initial sequence number.
 * RETURN VALUE:    A pseudo random number.
END DESCRIPTION ************************************************************/
UBaseType_t uxRand( void )
{       
        return( ( int ) (ReadCoreTimer() & 0x7fffUL));
}

    extern BaseType_t xApplicationGetRandomNumber( uint32_t * pulNumber )
    {
        *pulNumber = uxRand();
        return (pdTRUE);
    }
       
/*--------------------------End of main.c  -----------------------------------*/

