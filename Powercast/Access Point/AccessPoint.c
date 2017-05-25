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
* This program is based off of Microchip's MiWi Development Environment
* and demonstrates the capabilities of Powercast's P2110 Energy Harvester
* product.  This program is for receiving a MiWi packet from the End
* Point node and displaying the sensor information received on a
* computer terminal screen.
*  Updated
*this file is modified by NDSU ECE 209 to collect structure monitoring data
* Change History (Microchip Code):
*  Rev   Date         Author    Description
*  0.1   1/03/2008    yfy       Initial revision
*  2.0   4/15/2009    yfy       MiMAC and MiApp revision
*  3.1   5/28/2010    yfy       MiWi DE 3.1
*
* Change History (Powercast Code):
*  Ver   Date         Author    Description
*  1.0   10/14/2010   DWH       Initial revision
*  1.1   04/29/2011   DWH       Updated version for endpoint code revision
* 12    02/15/2017   RG        Updated version for Structure health monitoring
********************************************************************/

/* -- COMPILER DIRECTIVES -- */


/* -- INCLUDES -- */
#include "Console.h"
#include "ConfigApp.h"
#include "HardwareProfile.h"
#include "WirelessProtocols\MCHP_API.h"
#include <math.h>

/* -- DEFINES and ENUMS -- */
#define VBG_VAL         1228800UL   // VBG = 1.2V, VBG_VAL = 1200 mV * 1024 counts
#define T_BIAS          10000.0     // Bias res for temp sensor
#define B_CONST         3380.0      // B Constant of thermistor
#define R_0             10000.0     // Thermistor resistance at 25C
#define T_0             298.15      // Temp in kelvin at 25C
#define MYCHANNEL 25
#define CODE_VERSION 12

#define ADC_CALC_MAX_TIME_MS 200
#define RX_TIME 5
#define DEBUG
#define FIVE_SECONDS 5000
#define SIX_SECONDS 6000
#define THIRTY_SECONDS 30000
#define MAX_PACKET_SIZE 100
#define BUTTON_ONE 1
#define BUTTON_TWO 2
#define TOTAL_RESPONSE_BUFFERS 8

enum
{
    SLAVE_0_ID,
    SLAVE_1_ID,
    SLAVE_2_ID,
    SLAVE_3_ID,
    GLOBAL_ID = 0xFF
};

enum
{
    READ_ADC_CMD,
    REQ_STATUS_CMD
};


enum
{
    SLAVE_ID_INDEX,
    COMMAND_INDEX,
    MAX_THRESHOLD_INDEX,
    BUFFER_INDEX,
    ADC_VALUE_INDEX
};

/* -- TYPEDEFS and STRUCTURES -- */
typedef enum
{
    INACTIVE,
    REQ_READ_ADC,
    REQ_SLAVE_RES
} MASTER_STATES_E;

typedef enum
{
    SLAVE_NO_ACKNOWLEDGE,
    SLAVE_ACKNOWLEDGE
} SLAVE_RES_STATES_E;

/* -- STATIC AND GLOBAL VARIABLES -- */
static const BYTE kabySlaves[] = { SLAVE_0_ID, SLAVE_1_ID };

// Used for console
static char charBuffer[100];

/* -- STATIC FUNCTION PROTOTYPES -- */
static void scMainInit(void);
static void scTransmit(BYTE *pbyTxBuffer, BYTE byLength);
static BOOL scfReceive(RECEIVED_MESSAGE *stReceiveMessageBuffer);
static void scDoGlobalADCRequest(void);
static void scReqSlaveStatus(const BYTE kbySlaveID);
static void scPrintPacketToConsole(BYTE * byPacket, BYTE byLength);


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

    // Need to change some Console parameters for XLP16 board
#ifdef XLP16
    U2MODEbits.RXINV = 1;
    U2STAbits.UTXINV = 1;
#endif

    // LEDs are ACTIVE LOW, turn them both OFF
    LED_1 = 1;
    LED_2 = 1;

    // Initial Startup Display
    ConsolePutROMString((ROM char*)"\r\nNDSU ECE 209 Ali");
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
    if (MiApp_SetChannel(MYCHANNEL) == FALSE)
    {
        Printf("\r\nSelection of channel ");
        PrintDec(MYCHANNEL);
        Printf(" is not supported in current condition.\r\n");
        #if defined(__18CXX)
        return;
        #else
        return 0;
        #endif
    }

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
    BYTE bySlaveIndex = 0;
    RECEIVED_MESSAGE stReceivedMessage = (RECEIVED_MESSAGE) {0};
    MASTER_STATES_E eMasterStates = INACTIVE;
    BYTE byI;
    
    scMainInit();

    while(TRUE)
    {
        P2PTasks();
        
        switch(eMasterStates)
        {
            case INACTIVE:
                // Turn OFF LED 1 and 2 to indicate inactive
                LED_1 = 1;
                LED_2 = 1;
                if (ButtonPressed() == BUTTON_ONE)
                {
                    // Turn ON LED 1 to indicate ADC READ
                    LED_1 = 0;
                    eMasterStates = REQ_READ_ADC;
                }
                else if (ButtonPressed() == BUTTON_TWO) // Button 2 doesn't actually work
                {
                    // Turn ON LED 2 to indicate SLAVE RESPONSE
                    LED_2 = 0;
                    eMasterStates = REQ_SLAVE_RES;
                }
                break;

            case REQ_READ_ADC:
#ifdef DEBUG
ConsolePutROMString((ROM char *)"REQ_READ_ADC\r\n");
#endif /* ifndef DEBUG */
               // Time slave takes to calibrate
               //DelayMs(FIVE_SECONDS);

               scDoGlobalADCRequest();

               // Max time slave takes to read ADC
               DelayMs(SIX_SECONDS);

               eMasterStates = REQ_SLAVE_RES;
               break;

case REQ_SLAVE_RES:
#ifdef DEBUG
ConsolePutROMString((ROM char *)"REQ_SLAVE_RES\r\n");
#endif /* ifndef DEBUG */

                for(bySlaveIndex = 0; bySlaveIndex < sizeof(kabySlaves); bySlaveIndex++)
                {
                    scReqSlaveStatus(kabySlaves[bySlaveIndex]);
                                            
                    // Rx 5 packets from each slave
                    for (byI = 0; byI < TOTAL_RESPONSE_BUFFERS; byI++)
                    {                        
                        if (scfReceive(&stReceivedMessage))
                        {
                            if (stReceivedMessage.Payload[SLAVE_ID_INDEX] == bySlaveIndex)
                            {
                                switch ((SLAVE_RES_STATES_E)stReceivedMessage.Payload[COMMAND_INDEX])
                                {
                                    case SLAVE_NO_ACKNOWLEDGE:
                                        ConsolePutROMString((ROM char *)"SLAVE_NO_ACKNOWLEDGE\r\n");
                                        scPrintPacketToConsole(stReceivedMessage.Payload, ADC_VALUE_INDEX);
                                        break;

                                    case SLAVE_ACKNOWLEDGE:
                                        scPrintPacketToConsole(stReceivedMessage.Payload, MAX_PACKET_SIZE);
                                        break;

                                    default:
#ifdef DEBUG
ConsolePutROMString((ROM char *)"ERROR CASE: COMMAND_INDEX invalid\r\n");
#endif
                                        break;
                                }
                            }
                            else
                            {
#ifdef DEBUG
sprintf(charBuffer, "ERROR CASE: Invalid slave ID - Expected: %d | Received: %d\r\n\0", bySlaveIndex, stReceivedMessage.Payload[SLAVE_ID_INDEX]);
ConsolePutROMString((ROM char*)charBuffer);
#endif /* ifndef DEBUG */
                            }
                        }
                        else
                        {
#ifdef DEBUG
sprintf(charBuffer, "ERROR CASE: Timeout waiting for Packet %d from Node %d\r\n\0", byI, bySlaveIndex);
ConsolePutROMString((ROM char*)charBuffer);
#endif /* ifndef DEBUG */                   
                        }
                    }
                    Delay(100);
                }
                eMasterStates = INACTIVE;
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
    BYTE byTimeout = 0xFF;

    while (byTimeout > 0)
    {
        byTimeout--;
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
        DelayMs(RX_TIME);
    }

    return fRetVal;
}


/*----------------------------------------------------------------------------

@Prototype: static void ADCrequest(void)

@Description: Request ADC from all slaves registered to Global ID

@Parameters: void

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
 static void scDoGlobalADCRequest(void)
{
#ifdef DEBUG
ConsolePutROMString((ROM char *)"scDoGlobalADCRequest\r\n");
#endif /* ifndef DEBUG */

    const BYTE kabyDataBuffer[] = { GLOBAL_ID, READ_ADC_CMD };

    scTransmit((BYTE *)&kabyDataBuffer, sizeof(kabyDataBuffer));
}


/*----------------------------------------------------------------------------

@Prototype: static void ADCrequest(void)

@Description: Request ADC from all slaves registered to Global ID

@Parameters: void

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scReqSlaveStatus(const BYTE kbySlaveID)
{
#ifdef DEBUG
ConsolePut(kbySlaveID % 10 + '0');
ConsolePutROMString((ROM char *)" Node scReqSlaveStatus\r\n");
#endif /* ifndef DEBUG */

    const BYTE kabyDataBuffer[] = { kbySlaveID, REQ_STATUS_CMD };

    scTransmit((BYTE *)&kabyDataBuffer, sizeof(kabyDataBuffer));
}


/*----------------------------------------------------------------------------

@Prototype: static void scPrintPacketToConsole(BYTE * byPacket, BYTE byLength)

@Description: Pretty print some information to console

@Parameters: BYTE bySlaveID
             BYTE byMaxThreshold
             BYTE * byADCValues
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
            "Slave:%d | Command:%d | Threshold:%d | Buffer:%d | Values: \0",
            byPacket[SLAVE_ID_INDEX],
            byPacket[COMMAND_INDEX],
            byPacket[MAX_THRESHOLD_INDEX],
            byPacket[BUFFER_INDEX]);
    
    ConsolePutROMString((ROM char*)charBuffer);

    for (byIndex = ADC_VALUE_INDEX; byIndex < byLength; byIndex++)
    {
        sprintf(charBuffer, "%d | \0", (byPacket[byIndex] << 1));
        ConsolePutROMString((ROM char*)charBuffer);
    }
    ConsolePutROMString((ROM char*)"\r\n");
}
