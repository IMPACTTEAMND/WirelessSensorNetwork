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
#define RX_TIME 2
#define DEBUG
#define FIVE_SECONDS 5000
#define THIRTY_SECONDS 30000
#define MAX_PACKET_SIZE 20

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
    REQ_STATUS_CMD,
};


enum
{
    SLAVE_ID_INDEX,
    COMMAND_INDEX,
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

/* -- STATIC FUNCTION PROTOTYPES -- */
static void scMainInit(void);
static void scTransmit(BYTE *pbyTxBuffer, BYTE byLength);
static BOOL scfReceive(RECEIVED_MESSAGE *stReceiveMessageBuffer);
static void scDoGlobalADCRequest(void);
static void scReqSlaveStatus(const BYTE kbySlaveID);
static void scPrintADCValuesToConsole(BYTE bySlaveID, BYTE * byADCValues);


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
    ConsolePutROMString((ROM char*)"\r\nNDSU ECE 209");
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
    BYTE bySlaveRequestCount = 0;
    RECEIVED_MESSAGE stReceivedMessage = (RECEIVED_MESSAGE) {0};
    MASTER_STATES_E eMasterStates = INACTIVE;
    BYTE byADCVal[MAX_PACKET_SIZE];
    BYTE byIndex;
    
    scMainInit();

    while(TRUE)
    {
        switch(eMasterStates)
        {
            case INACTIVE:
                // Turn OFF LED 1 and 2 to indicate inactive
                LED_1 = 1;
                LED_2 = 1;
                bySlaveIndex = 0;
                bySlaveRequestCount = 0;
                if (ButtonPressed())
                {
                    // Turn ON LED 1 to indicate active
                    LED_1 = 0;
                    eMasterStates = REQ_READ_ADC;
                }
                break;
                
            case REQ_READ_ADC:
#ifdef DEBUG
ConsolePutROMString((ROM char *)"REQ_READ_ADC\r\n");
#endif /* ifndef DEBUG */
               // Time slave takes to calibrate
               DelayMs(FIVE_SECONDS);
#ifdef DEBUG
ConsolePutROMString((ROM char *)"5sec calibration done\r\n");
#endif /* ifndef DEBUG */

               scDoGlobalADCRequest();
               
               // Max time slave takes to read ADC
               DelayMs(THIRTY_SECONDS);
#ifdef DEBUG
ConsolePutROMString((ROM char *)"30sec ADC read done\r\n");
#endif /* ifndef DEBUG */
               eMasterStates = REQ_SLAVE_RES;
               break;

            case REQ_SLAVE_RES:
#ifdef DEBUG
ConsolePutROMString((ROM char *)"REQ_SLAVE_RES\r\n");
#endif /* ifndef DEBUG */
                while (bySlaveIndex < sizeof(kabySlaves))
                {
                    scReqSlaveStatus(kabySlaves[bySlaveIndex]);
                    
                    if (scfReceive(&stReceivedMessage))
                    {   
#ifdef DEBUG
ConsolePutROMString((ROM char *)"Rx'd Slave Response\r\n");
#endif /* ifndef DEBUG */
                        if (stReceivedMessage.Payload[SLAVE_ID_INDEX] == bySlaveIndex)
                        {
                            switch ((SLAVE_RES_STATES_E)stReceivedMessage.Payload[COMMAND_INDEX])
                            {
                                case SLAVE_NO_ACKNOWLEDGE:
                                    ConsolePutROMString((ROM char *)"ERROR CASE: SLAVE_NO_ACKNOWLEDGE\r\n");
                                    break;

                                case SLAVE_ACKNOWLEDGE:
                                    ConsolePutROMString((ROM char *)"SLAVE_ACKNOWLEDGE\r\n");
                                    for (byIndex = 0; byIndex < MAX_PACKET_SIZE-2; byIndex++)
                                    {
                                        byADCVal[byIndex] = stReceivedMessage.Payload[byIndex + ADC_VALUE_INDEX];
                                    }
                                    scPrintADCValuesToConsole(bySlaveIndex, byADCVal);
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
ConsolePutROMString((ROM char *)"ERROR CASE: Invalid Slave ID\r\n");
#endif /* ifndef DEBUG */
                            break;
                        }
                        bySlaveIndex++;
                    }
                    else
                    {
#ifdef DEBUG
ConsolePutROMString((ROM char *)"ERROR CASE: Timed out waiting for slave response\r\n");
#endif /* ifndef DEBUG */
                        // Request again, upto 5 times
                        if (bySlaveRequestCount < 5)
                        {
                            // Stay in this state
                            
                            // Increment slave request count
                            bySlaveRequestCount++;
                            
                            // Delay to re-request
                            DelayMs(1000);
                        }
                        // If we have other slaves, request them
                        else if (bySlaveIndex < sizeof(kabySlaves))
                        {
                            // Stay in this state 
                            
                            // Reset the slave request count
                            bySlaveRequestCount = 0;
                            bySlaveIndex++;
                        }
                        else
                        {
                            break;
                        }
                    }
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
    BYTE byTimeout = 0xFF;

    // Timeout after 255ms
    while (byTimeout > 0)
    {
        byTimeout--;
        if (MiApp_MessageAvailable())
        {
            * stReceiveMessageBuffer = rxMessage;
            MiApp_DiscardMessage();
            break;
        }
        DelayMs(RX_TIME);
    }

    return (byTimeout > 0);
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
    const BYTE kabyDataBuffer[] = { kbySlaveID, REQ_STATUS_CMD };

    scTransmit((BYTE *)&kabyDataBuffer, sizeof(kabyDataBuffer));
}


/*----------------------------------------------------------------------------
 
@Prototype: static void scPrintADCValuesToConsole(BYTE bySlaveID, BYTE * byADCValues);
 
@Description: Pretty print some information to console

@Parameters: BYTE bySlaveID
             BYTE * byADCValues

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scPrintADCValuesToConsole(BYTE bySlaveID, BYTE * byADCValues)
{
    char str[100];
    BYTE byIndex;
    
    ConsolePutROMString((ROM char*)"Node: ");
    ConsolePut(bySlaveID % 10 + '0');
    ConsolePutROMString((ROM char*)" | ");
    ConsolePutROMString((ROM char*)"Values: ");

    for (byIndex = 0; byIndex < MAX_PACKET_SIZE-2; byIndex++)
    {
        sprintf(str, "%d", (byADCValues[byIndex] << 1));
        ConsolePutROMString((ROM char*)str);
        ConsolePutROMString((ROM char*)" | ");
    }
    ConsolePutROMString((ROM char*)"\r\n");
}

