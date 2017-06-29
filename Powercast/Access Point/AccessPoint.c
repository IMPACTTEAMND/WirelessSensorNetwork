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
#define DEBUG

/* -- GLOBAL VARIABLES -- */
volatile WORD_BYTE_U gvuwTimer;

/* -- STATIC VARIABLES -- */
static const BYTE kabySlaves[] = 
{ 
    SLAVE_0_ID, 
    SLAVE_1_ID, 
    SLAVE_2_ID,
    SLAVE_3_ID,
    SLAVE_4_ID,
    SLAVE_5_ID,
    SLAVE_6_ID,
    SLAVE_7_ID,
    SLAVE_8_ID,
    SLAVE_9_ID,
    SLAVE_10_ID
};

// Used for console
static char charBuffer[200];

/* -- STATIC FUNCTION PROTOTYPES -- */
static void scMainInit(void);
static void scTransmit(BYTE *pbyTxBuffer, BYTE byLength);
static BOOL scfReceive(RECEIVED_MESSAGE *stReceiveMessageBuffer);
static void scDoCommand(BYTE bySlaveID, BYTE byCommand);
static void scReqSlaveBuffer(BYTE bySlaveID, BYTE byBuffer);
static void scReqSlaveADCBuffers();
static void scReqSlavePositionBuffer();
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
    _T1IF = 0;  // Clear Timer 1 interrupt flag
    TMR1 = PULSES;
    
    gvuwTimer.wOrd++;
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
	// Set up Timer1
  	T1CON = 0x0000; 		// stops the timer1 and reset control flag
  	TMR1 = PULSES;
  	IPC0bits.T1IP = HIGHEST_PRIORITY; 	// setup Timer1 interrupt for desired priority level
	IEC0bits.T1IE = 1; 		// enable Timer1 interrupts	
	T1CON = 0x8010;		 	// enable timer1 with prescalar of 1:8 
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
    gvuwTimer.wOrd = 0;
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
    COMMANDS_E eMasterCommands = INVALID_CMD;
    BYTE byButtonPressCount = 0;
    BYTE byButtonPressWaitDelay = 0xFF;
    
    scMainInit();

    while(TRUE)
    {
        /* This function maintains the operation of the stack
           It should be called as often as possible. */
        P2PTasks();

        switch(eMasterCommands)
        {
            case DO_CALIBRATION_CMD:
                scDoCommand(GLOBAL_ID, DO_CALIBRATION_CMD);
                eMasterCommands = INVALID_CMD;
                break;
                
            case DO_READ_ADC_CMD:
               scDoCommand(GLOBAL_ID, DO_READ_ADC_CMD);

               gvuwTimer.wOrd = 0;
               while (gvuwTimer.wOrd < TIME_TO_MEASURE_ADC_MASTER);

               eMasterCommands = INVALID_CMD;
               break;
            
            case DO_MEASURE_POSITION_CMD:
                scDoCommand(GLOBAL_ID, DO_MEASURE_POSITION_CMD);
                eMasterCommands = INVALID_CMD;
                break;
            
            case REQ_BUFFER_CMD:
                scReqSlaveADCBuffers();
                eMasterCommands = INVALID_CMD;
                break; 
                
            case REQ_POSITION_TIMER_CMD:
                scReqSlavePositionBuffer();
                eMasterCommands = INVALID_CMD;
                break;                
                
            case INVALID_CMD:
            default:
                // Turn OFF LED 1 and 2 to indicate inactive
                LED_1 = 1;
                LED_2 = 1;
                if (ButtonPressed() == BUTTON_ONE)
                {
                    // Wait ~3 Seconds and count the amount of button presses
                    while (byButtonPressWaitDelay--)
                    {
                        DelayMs(15);
                        if (ButtonPressed() == BUTTON_ONE)
                        {
                            byButtonPressCount++;
                            // Toggle LED 1 and LED 2
                            LED_1 ^= 1;
                            LED_2 ^= 1;
                        }
                    }
                    if(byButtonPressCount < INVALID_CMD)
                    {
                        // Turn ON LED 1 to indicate eMasterStates
                        LED_1 = 0;
                        LED_2 = 1;
                        eMasterCommands = byButtonPressCount;
                    }
                    else
                    {
                        #ifdef DEBUG
                        sprintf(charBuffer,
                                "Too many button presses:%u\r\n",
                                byButtonPressCount);
                        ConsolePutROMString((ROM char *)charBuffer);
                        #endif /* ifndef DEBUG */
                    }
                    byButtonPressCount = 0;
                }
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

    gvuwTimer.wOrd = 0;
    while (gvuwTimer.wOrd < FIVE_HUNDRED_MS)
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

@Prototype:  static void scDoCommand(BYTE bySlaveID, BYTE byCommand)

@Description: Request a command from slave

@Parameters: BYTE bySlaveID
             BYTE byCommand

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
06/21/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scDoCommand(BYTE bySlaveID, BYTE byCommand)
{
    #ifdef DEBUG
    sprintf(charBuffer,
            "Slave:%u Command:%u\r\n",
            bySlaveID,
            byCommand);
    ConsolePutROMString((ROM char *)charBuffer);
    #endif /* ifndef DEBUG */

    BYTE abyDataBuffer[] = { bySlaveID, byCommand };

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
            "Request Slave:%u Buffer:%u\r\n",
            bySlaveID,
            byBuffer);
    ConsolePutROMString((ROM char*)charBuffer);
    #endif /* ifndef DEBUG */

    // TODO: Make generic to map directly to INDEX
    BYTE abyDataBuffer[] = { bySlaveID, REQ_BUFFER_CMD, byBuffer };

    scTransmit((BYTE *)&abyDataBuffer, sizeof(abyDataBuffer));
}


/*----------------------------------------------------------------------------

@Prototype: static void scReqSlavePositionBuffer()

@Description: Request position buffer from all slaves

@Parameters: void

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
06/21/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scReqSlavePositionBuffer()
{
    RECEIVED_MESSAGE stReceivedMessage = (RECEIVED_MESSAGE){0};
    BYTE bySlaveIndex = 0;
    WORD_BYTE_U uwMax;
    WORD_BYTE_U uwMin;
    WORD_BYTE_U uwAvg;
    WORD_BYTE_U uwTime;
    WORD_BYTE_U uwTicks;
    
    for(bySlaveIndex = 0; bySlaveIndex < NUMBER_OF_SLAVES; bySlaveIndex++)
    {
        scDoCommand(kabySlaves[bySlaveIndex], REQ_POSITION_TIMER_CMD);

        if (scfReceive(&stReceivedMessage))
        {
            if (stReceivedMessage.Payload[SLAVE_ID_INDEX] == bySlaveIndex)
            {
                uwMax.abyTe[0] = stReceivedMessage.Payload[MAX_INDEX_L];
                uwMax.abyTe[1] = stReceivedMessage.Payload[MAX_INDEX_H];
                uwMin.abyTe[0] = stReceivedMessage.Payload[MIN_INDEX_L];
                uwMin.abyTe[1] = stReceivedMessage.Payload[MIN_INDEX_H];
                uwAvg.abyTe[0] = stReceivedMessage.Payload[AVER_INDEX_L];
                uwAvg.abyTe[1] = stReceivedMessage.Payload[AVER_INDEX_H];
                uwTime.abyTe[0] = stReceivedMessage.Payload[TIME_INDEX_L];
                uwTime.abyTe[1] = stReceivedMessage.Payload[TIME_INDEX_H];
                uwTicks.abyTe[0] = stReceivedMessage.Payload[TICKS_INDEX_L];
                uwTicks.abyTe[1] = stReceivedMessage.Payload[TICKS_INDEX_H];
                 
                sprintf(charBuffer, 
                        "SlaveID:%u Max:%u Min:%u Ave:%u Time:%u PosTicks:%u\r\n",
                        stReceivedMessage.Payload[SLAVE_INDEX],
                        uwMax.wOrd,
                        uwMin.wOrd,
                        uwAvg.wOrd,
                        uwTime.wOrd,
                        uwTicks.wOrd);
                ConsolePutROMString((ROM char*)charBuffer);
            }
            else
            {
                #ifdef DEBUG
                sprintf(charBuffer,
                        "ERROR CASE: Invalid slave ID - Expected:%u | Received:%u\r\n",
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
                    "ERROR CASE: Timeout waiting for Slave:%u\r\n",
                    bySlaveIndex);
            ConsolePutROMString((ROM char*)charBuffer);
            #endif /* ifndef DEBUG */ 
        }
        // Cool down period between slave RX/TX
        DelayMs(5);
    }
}


/*----------------------------------------------------------------------------

@Prototype: static void scReqSlaveADCBuffers()

@Description: Request ADC buffers all slaves registered to ID

@Parameters: void

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

    for(bySlaveIndex = 0; bySlaveIndex < NUMBER_OF_SLAVES; bySlaveIndex++)
    {
        sprintf(charBuffer, "\r\nSlaveID:%u\r\n", bySlaveIndex);
        ConsolePutROMString((ROM char*)charBuffer);
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
                            case INVALID_STATUS:
                            case READ_ADC_FAILED:
                                sprintf(charBuffer,
                                        "Slave Status:%u - SlaveID:%u Buffer:%u\r\n",
                                        stReceivedMessage.Payload[STATUS_INDEX],
                                        bySlaveIndex,
                                        byBufferIndex);
                                ConsolePutROMString((ROM char*)charBuffer);
                                scPrintPacketToConsole(stReceivedMessage.Payload, ADC_VALUE_INDEX_H);
                                break;

                            case READ_ADC_PASSED:
                                scPrintPacketToConsole(stReceivedMessage.Payload, MAX_PACKET_SIZE);
                                break;

                            default:
                                #ifdef DEBUG
                                sprintf(charBuffer,
                                        "ERROR CASE: Unknown Status:%u\r\n",
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
                                "ERROR CASE: Invalid buffer index - Expected:%u | Received:%u\r\n",
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
                            "ERROR CASE: Invalid slave ID - Expected:%u | Received:%u\r\n",
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
                        "ERROR CASE: Timeout waiting for Slave:%u Buffer:%u\r\n",
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
    WORD_BYTE_U uwMax;
    WORD_BYTE_U uwMin;
    WORD_BYTE_U uwAvg;
    WORD_BYTE_U uwAdcVal;
    
    uwMax.abyTe[0] = byPacket[MAX_THRESHOLD_INDEX_L];
    uwMax.abyTe[1] = byPacket[MAX_THRESHOLD_INDEX_H];
    uwMin.abyTe[0] = byPacket[MIN_THRESHOLD_INDEX_L];
    uwMin.abyTe[1] = byPacket[MIN_THRESHOLD_INDEX_H];
    uwAvg.abyTe[0] = byPacket[AVERAGE_INDEX_L];
    uwAvg.abyTe[1] = byPacket[AVERAGE_INDEX_H];
    
    #ifdef DEBUG
    sprintf(charBuffer,
            "Slv:%u Buf:%u Sta:%u Max:%u Min:%u Ave:%u Adc: ",
            byPacket[SLAVE_ID_INDEX],
            byPacket[STATUS_INDEX],
            byPacket[COMMAND_INDEX],
            uwMax.wOrd,
            uwMin.wOrd,
            uwAvg.wOrd);
    ConsolePutROMString((ROM char*)charBuffer);
    #endif 
    
    for (byIndex = ADC_VALUE_INDEX_H; byIndex < byLength; byIndex++)
    {
        uwAdcVal.abyTe[1] = byPacket[byIndex];
        byIndex++;
        uwAdcVal.abyTe[0] = byPacket[byIndex];
        sprintf(charBuffer, "%u,", uwAdcVal.wOrd);
        ConsolePutROMString((ROM char*)charBuffer);
    }
    
    #ifdef DEBUG
    ConsolePutROMString((ROM char*)"\r\n");
    #endif
}

