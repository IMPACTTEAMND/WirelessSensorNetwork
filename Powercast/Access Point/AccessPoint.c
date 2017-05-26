/********************************************************************
* FileName:     AccessPoint.c
* Dependencies: none
* Processor:    PIC24 (tested)
* Complier:     Microchip C30 v3.23 or higher (tested)
*
* Processor:    PIC18, PIC32, dsPIC30, dsPIC33 (untested)
* Complier:     Microchip C18 v3.04 or higher (untested)
*               Microchip C32 v1.02 or higher
*


*********************************************************************
* File Description:
*
********************************************************************/

/* -- COMPILER DIRECTIVES -- */


/* -- INCLUDES -- */
#include "WirelessProtocols\P2P\P2P.h"
#include "Console.h"
#include "ConfigApp.h"
#include "HardwareProfile.h"
#include "WirelessProtocols\MCHP_API.h"
#include "TimeDelay.h" 
#include "..\SharedFiles\Master_Slave_Config.h"
#include <stdint.h>
#include <math.h>


/* -- DEFINES and ENUMS -- */

/* Comment/Uncomment to enable/disable debug/*/
//#define DEBUG


/* -- GLOBAL VARIABLES -- */
volatile DWORD gdwRxTicks;
volatile DWORD gdwADCTicks;

/* -- STATIC VARIABLES -- */
static const BYTE kabySlaves[] = { SLAVE_0_ID, SLAVE_1_ID, SLAVE_2_ID };

// Used for console
static char charBuffer[100];

/* -- STATIC FUNCTION PROTOTYPES -- */
static void scMainInit(void);
static void scTransmit(BYTE *pbyTxBuffer, BYTE byLength);
static BOOL scfReceive(RECEIVED_MESSAGE *stReceiveMessageBuffer);
static void scADCRequest(BYTE bySlaveID);
static void scReqSlaveBuffer(BYTE bySlaveID, BYTE byBuffer);
static void scReqSlaveADCBuffers();
static void scPrintPacketToConsole(BYTE * byPacket, BYTE byLength);


/*********************************************************************
* Function:         void T1Interrupt(void)
*
* PreCondition:     none
*
* Input:            none
*
* Output:           none
*
* Side Effects:     none
*
* Overview:         Interrupt function for Timer1.  Set up to time out
*                   after 100 us, updates total time
*
* Note:
**********************************************************************/
void _ISRFAST __attribute__((interrupt, auto_psv)) _T1Interrupt(void)
{
    gdwRxTicks++;
    gdwADCTicks++;
    _T1IF = 0;  // Clear Timer 1 interrupt flag
    TMR1 = PULSES;
    return;
}


/*----------------------------------------------------------------------------

@Prototype: static void scTimerInit (void)

@Description: initiate the timer interrupt
@Parameters: None

@Returns: None

@Revision History:
DATE             NAME               REVISION COMMENT
04/18/2017       Ruisi Ge           Initial Revision
*----------------------------------------------------------------------------*/
static void scTimerInterruptInit (void)
{
    T1CON = 0x0000; // stops the timer1 and reset control flag
    TMR1 = PULSES;          
    _T1IP = HIGHEST_PRIORITY;     // setup Timer1 interrupt for desired priority level
    _T1IE = 1; // enable Timer1 interrupts
    T1CON = 0x8010; // enable timer1 with prescalar of 1:8
}


/*----------------------------------------------------------------------------

@Description: The  Application main entry point initialization

@Parameters: void

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scMainInit(void)
{
    BoardInit();
    ConsoleInit();
    scTimerInterruptInit();
    
    // Need to change some Console parameters for XLP16 board
#ifdef XLP16
    U2MODEbits.RXINV = 1;
    U2STAbits.UTXINV = 1;
#endif

    // LEDs are ACTIVE LOW, turn them both OFF
    LED_1 = 1;
    LED_2 = 1;

    // Initial Startup Display
    ConsolePutROMString((ROM char*)"\r\nNDSU ECE 209 Ali Haidous");
    ConsolePutROMString((ROM char*)"\r\n Wireless sensor network ");
    ConsolePutROMString((ROM char*)"\r\nVersion ");
    ConsolePut((CODE_VERSION / 10) % 10 + '0');
    ConsolePut('.');
    ConsolePut(CODE_VERSION % 10 + '0');
    ConsolePutROMString((ROM char*)"\r\n");


    /*******************************************************************/
    // Function MiApp_ProtocolInit initialize the protocol stack. The
    // only input parameter indicates if previous network configuration
    // should be restored. In this simple example, we assume that the
    // network starts from scratch.
    /*******************************************************************/
    MiApp_ProtocolInit(FALSE);
    // Set default channel
    MiApp_SetChannel(RF_TRANSCEIVER_CHANNEL);


    /*******************************************************************/
    // Function MiApp_ConnectionMode defines the connection mode. The
    // possible connection modes are:
    //  ENABLE_ALL_CONN:    Enable all kinds of connection
    //  ENABLE_PREV_CONN:   Only allow connection already exists in
    //                      connection table
    //  ENABL_ACTIVE_SCAN_RSP:  Allow response to Active scan
    //  DISABLE_ALL_CONN:   Disable all connections.
    /*******************************************************************/
    MiApp_ConnectionMode(ENABLE_ALL_CONN);
    
    /* Initialize global and static variables */
    gdwRxTicks = 0;
    gdwADCTicks = 0;
}


/*----------------------------------------------------------------------------

@Description: The  Application main entry point

@Parameters: void

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
int main(void)
{
    MASTER_STATES_E eMasterStates = INACTIVE;

    scMainInit();

    while(TRUE)
    {
        /* This function maintains the operation of the stack
           It should be called as often as possible. */
        P2PTasks();

        switch(eMasterStates)
        {
            case INACTIVE:
                // Turn OFF LED 1 and 2 to indicate inactive
                LED_1 = 1;
                LED_2 = 1;
                if (ButtonPressed() == BUTTON_ONE)
                {
                    // Turn ON LED 1 to indicate ADC READ begins
                    LED_1 = 0;
                    eMasterStates = REQ_READ_ADC;
                    //eMasterStates = REQ_SLAVE_ADC_BUFFERS;
                }
                else if (ButtonPressed() == BUTTON_TWO) // Button 2 doesn't actually work
                {
                    // Turn ON LED 2 to indicate SLAVE RESPONSE
                    LED_2 = 0;
                    eMasterStates = REQ_SLAVE_ADC_BUFFERS;
                }
                break;

            case REQ_READ_ADC:
               scADCRequest(GLOBAL_ID);

               gdwADCTicks = 0;
               while (gdwADCTicks < TIME_TO_MEASURE_ADC);

               eMasterStates = REQ_SLAVE_ADC_BUFFERS;
               break;

            case REQ_SLAVE_ADC_BUFFERS:
                scReqSlaveADCBuffers();
                
                //eMasterStates = INACTIVE;
                eMasterStates = REQ_READ_ADC;
                break;

            default:
                // Error case
                eMasterStates = INACTIVE;
                break;
        }
    }

    return 0;
}


/*----------------------------------------------------------------------------

@Prototype: static void scTransmit(BYTE * pbyTxBuffer, BYTE byLength)

@Description: Request ADC from all slaves

@Parameters: BYTE * pbyTxBuffer - Pointer to the buffer of bytes to transmit
             BYTE byLength - The length of bytes

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scTransmit(BYTE * pbyTxBuffer, BYTE byLength)
{
    BYTE byI;

    // Clear the transmit buffer
    MiApp_FlushTx();

    // Write new frame to transmit buffer
    for (byI = 0; byI < byLength; byI++)
    {
         MiApp_WriteData(pbyTxBuffer[byI]);
    }

    #ifdef DEBUG
    ConsolePutROMString((ROM char *)"<<<<Tx\r\n");
    #endif /* ifndef DEBUG */

    // Broadcast packet from transmit buffer
    MiApp_BroadcastPacket(FALSE);
}


/*----------------------------------------------------------------------------

@Prototype: static BOOL scfReceive(void)

@Description: Wait until we receive a packet, then timeout if we receive nothing

@Parameters: RECEIVED_MESSAGE * stReceiveMessageBuffer - Out parameter for the
             received message.

@Returns: BOOL - If a timeout occurs


@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static BOOL scfReceive(RECEIVED_MESSAGE * stReceiveMessageBuffer)
{
    BOOL fRetVal = FALSE;

    // Reset Rx timer
    gdwRxTicks = 0;
    
    while (gdwRxTicks < FIVE_HUNDRED_MS)
    {
        if (MiApp_MessageAvailable())
        {
            #ifdef DEBUG
            ConsolePutROMString((ROM char *)"Rx>>>>\r\n");
            #endif /* ifndef DEBUG */

            *stReceiveMessageBuffer = rxMessage;
            MiApp_DiscardMessage();
            fRetVal = TRUE;
            break;
        }
    }

    return fRetVal;
}


/*----------------------------------------------------------------------------

@Prototype:  static void scADCRequest(void)

@Description: Request ADC from slave registered to ID

@Parameters: BYTE bySlaveID

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scADCRequest(BYTE bySlaveID)
{
    #ifdef DEBUG
    sprintf(charBuffer,
            "ADC Request Slave:%d\r\n",
            bySlaveID);
    ConsolePutROMString((ROM char *)charBuffer);
    #endif /* ifndef DEBUG */

    // TODO: Make generic to map directly to INDEX
    BYTE abyDataBuffer[] = { bySlaveID, READ_ADC_CMD };

    scTransmit((BYTE *)&abyDataBuffer, sizeof(abyDataBuffer));
}


/*----------------------------------------------------------------------------

@Prototype: static void scReqSlaveBuffer(BYTE bySlaveID, BYTE byBuffer)

@Description: Request buffer from slave registered to ID

@Parameters: BYTE bySlaveID 
             BYTE byBuffer

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scReqSlaveBuffer(BYTE bySlaveID, BYTE byBuffer)
{
    #ifdef DEBUG
    sprintf(charBuffer,
            "Request Slave:%d Buffer:%d\r\n",
            bySlaveID,
            byBuffer);
    ConsolePutROMString((ROM char*)charBuffer);
    #endif /* ifndef DEBUG */

    // TODO: Make generic to map directly to INDEX
    BYTE abyDataBuffer[] = { bySlaveID, REQ_BUFFER_CMD, byBuffer };

    scTransmit((BYTE *)&abyDataBuffer, sizeof(abyDataBuffer));
}


/*----------------------------------------------------------------------------

@Prototype: static void scReqSlaveBuffer(BYTE bySlaveID, BYTE byBuffer)

@Description: Request ADC buffers from slave registered to ID

@Parameters: BYTE bySlaveID 
             BYTE byBuffer

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
05/25/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scReqSlaveADCBuffers()
{
    RECEIVED_MESSAGE stReceivedMessage = (RECEIVED_MESSAGE){0};
    BYTE byBufferIndex;
    BYTE bySlaveIndex;

    for(bySlaveIndex = 0; bySlaveIndex < sizeof(kabySlaves); bySlaveIndex++)
    {
        for (byBufferIndex = 0; byBufferIndex < TOTAL_RESPONSE_BUFFERS; byBufferIndex++)
        {
            scReqSlaveBuffer(kabySlaves[bySlaveIndex], byBufferIndex);

            if (scfReceive(&stReceivedMessage))
            {
                if (stReceivedMessage.Payload[SLAVE_ID_INDEX] == bySlaveIndex)
                {
                    if (stReceivedMessage.Payload[BUFFER_INDEX] == byBufferIndex)
                    {
                        switch ((SLAVE_STATUS_E)stReceivedMessage.Payload[STATUS_INDEX])
                        {
                            case READ_ADC_FAILED:
                                ConsolePutROMString((ROM char *)"Read ADC Failed\r\n");
                                scPrintPacketToConsole(stReceivedMessage.Payload,
                                                       ADC_VALUE_INDEX);
                                break;

                            case READ_ADC_PASSED:
                                scPrintPacketToConsole(stReceivedMessage.Payload,
                                                       MAX_PACKET_SIZE);
                                break;

                            default:
                                #ifdef DEBUG
                                sprintf(charBuffer,
                                        "ERROR CASE: Invalid Status:%d\r\n",
                                        stReceivedMessage.Payload[STATUS_INDEX]);
                                ConsolePutROMString((ROM char *)charBuffer);
                                #endif
                                break;
                        }
                    }
                    else
                    {
                        #ifdef DEBUG
                        sprintf(charBuffer,
                                "ERROR CASE: Invalid buffer index - Expected:%d | Received:%d\r\n",
                                byBufferIndex,
                                stReceivedMessage.Payload[BUFFER_INDEX]);
                        ConsolePutROMString((ROM char*)charBuffer);
                        #endif /* ifndef DEBUG */
                    }
                }
                else
                {
                    #ifdef DEBUG
                    sprintf(charBuffer,
                            "ERROR CASE: Invalid slave ID - Expected:%d | Received:%d\r\n",
                            bySlaveIndex,
                            stReceivedMessage.Payload[SLAVE_ID_INDEX]);
                    ConsolePutROMString((ROM char*)charBuffer);
                    #endif /* ifndef DEBUG */
                }
            }
            else
            {
                #ifdef DEBUG
                sprintf(charBuffer,
                        "ERROR CASE: Timeout waiting for Slave:%d Buffer:%d\r\n",
                        bySlaveIndex,
                        byBufferIndex);
                ConsolePutROMString((ROM char*)charBuffer);
                #endif /* ifndef DEBUG */
            }
            
            // Cool down period between slave RX/TX
            DelayMs(5);
        }
    }
}


/*----------------------------------------------------------------------------

@Prototype: static void scPrintPacketToConsole(BYTE * byPacket, BYTE byLength)

@Description: Pretty print some information to console

@Parameters: BYTE * byPacket
             BYTE byLength

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scPrintPacketToConsole(BYTE * byPacket, BYTE byLength)
{
    BYTE byIndex;

    sprintf(charBuffer,
            "Slave:%d Buffer:%d Command:%d Threshold:%d Average:%d Values: ",
            byPacket[SLAVE_ID_INDEX],
            byPacket[BUFFER_INDEX],
            byPacket[COMMAND_INDEX],
            byPacket[MAX_THRESHOLD_INDEX],
            byPacket[AVERAGE_INDEX]);

    ConsolePutROMString((ROM char*)charBuffer);

    for (byIndex = ADC_VALUE_INDEX; byIndex < byLength; byIndex++)
    {
        sprintf(charBuffer, "%d ", (byPacket[byIndex] << 1));
        ConsolePutROMString((ROM char*)charBuffer);
    }
    ConsolePutROMString((ROM char*)"\r\n");
}

