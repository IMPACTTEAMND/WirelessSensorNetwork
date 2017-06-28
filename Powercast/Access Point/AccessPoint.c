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


/* -- GLOBAL VARIABLES -- */

// Used for console
static char charBuffer[200];

/* -- STATIC FUNCTION PROTOTYPES -- */
static void scMainInit(void);
static BOOL scfReceive(RECEIVED_MESSAGE *stReceiveMessageBuffer);


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
}


/*----------------------------------------------------------------------------

@Description: The  Application main entry point

@Parameters: void

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
RECEIVED_MESSAGE stReceivedMessage = (RECEIVED_MESSAGE){0};
QWORD_BYTES qwSlaveTicks;
QWORD_BYTES qwSlaveTime;
int main(void)
{
    BYTE byIndex = 0;
    
    scMainInit();

    while(TRUE)
    {
        if (scfReceive(&stReceivedMessage))
        {
            for (byIndex = 0; byIndex < 8; byIndex++)
            {
                qwSlaveTicks.abyTe[byIndex] = stReceivedMessage.Payload[byIndex];
                qwSlaveTime.abyTe[byIndex] = stReceivedMessage.Payload[byIndex + 8]; 
            }
            
            
            sprintf(charBuffer, 
                    "SlaveTicks:%llu SlaveTime:%llu RealTime:%llu\r\n",
                     qwSlaveTicks.qwOrd,
                    qwSlaveTime.qwOrd,
                    (qwSlaveTime.qwOrd * 65535)/8000000);
            ConsolePutROMString((ROM char*)charBuffer);
        }
    }

    return 0;
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

    if (MiApp_MessageAvailable())
    {
        *stReceiveMessageBuffer = rxMessage;
        MiApp_DiscardMessage();
        fRetVal = TRUE;
    }

    return fRetVal;
}