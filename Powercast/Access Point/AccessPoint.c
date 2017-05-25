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
#include <stdint.h>
#include <math.h>

/* -- DEFINES and ENUMS -- */
#define VBG_VAL                     1228800UL   // VBG = 1.2V, VBG_VAL = 1200 mV * 1024 counts
#define T_BIAS                      10000.0     // Bias res for temp sensor
#define B_CONST                     3380.0      // B Constant of thermistor
#define R_0                         10000.0     // Thermistor resistance at 25C
#define T_0                         298.15      // Temp in kelvin at 25C
#define MYCHANNEL                   25
#define CODE_VERSION                15
#define MAX_PACKET_SEQUENCE         4       
#define MAX_DATA_SIZE               75          // Max data packet= (MAX_PACKET_SIZE-HEADERCOOMANDSIZE)/2
#define ADC_CALC_MAX_TIME_MS        100
#define FIVE_SECONDS                5000
#define RX_TIME                     2
#define HEADERCOOMANDSIZE           3
#define TIMEHEADERSIZE              4
#define THRESHHOLDVALUEHEADER       2

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
    CALC_ADC_CMD,
    REQ_STATUS_CMD,
    REQ_MISS_MESSAGE_CMD,    
};


enum
{
    SLAVE_ID_INDEX,
    COMMAND_INDEX,
    SEQUENCE_INDEX,
    TIME_1_BYTE,
    TIME_2_BYTE,
    TIME_3_BYTE,
    TIME_4_BYTE,
    MAX_VALUE_HIGH_BYTE,
    MAX_VALUE_LOW_BYTE,
};

/* -- TYPEDEFS and STRUCTURES -- */
typedef enum
{
    INACTIVE,
    REQ_ADC_CALC,
    REQ_SLAVE_RES,
} MASTER_STATES_E;
typedef enum
{
    REQ_FULL_MESSAGE,
    CHECK_MESSAGE,
    REQ_MISS_MESSAGE
}REQUEST_RESPOND_STATES_E;

typedef enum
{
    SLAVE_NO_ACKNOWLEDGE,
    SLAVE_ACKNOWLEDGE
} SLAVE_RES_STATES_E;

/* -- STATIC AND GLOBAL VARIABLES -- */
 static char str[100];
 static const BYTE kabySlaves[] = { SLAVE_0_ID, SLAVE_1_ID };
 static unsigned int  ADCValue[MAX_PACKET_SEQUENCE][MAX_DATA_SIZE] ;
 static unsigned int  MaxThreshouldValue;                     // Max threshhold value received 
 static BYTE MaxThreshouldValueHighByte;
 static BYTE MaxThreshouldValueLowByte;
 static unsigned long TotalHundredMicroseconds; 
 static BYTE byHundredMicroseconds1stByte;                 // First Byte for Milliseconds 
 static BYTE byHundredMicroseconds2ndByte;                // Second Byte for Milliseconds
 static BYTE byHundredMicroseconds3rdByte;               //  Third Byte for Milliseconds
 static BYTE byHundredMicroseconds4thByte;              //  Fourth Byte for Milliseconds
/* -- STATIC FUNCTION PROTOTYPES -- */
static void scMainInit(void);
static void scTransmit(BYTE *pbyTxBuffer, BYTE byLength);
static BOOL scfReceive(RECEIVED_MESSAGE *stReceiveMessageBuffer);
static void scDoGlobalADCRequest(void);
static void scReqSlaveStatus(const BYTE kbySlaveID, BYTE byPacketSequence);
static void scPrintConsole(BYTE bySlaveID,BYTE PacketSequence);
static void scCollectandSortMessage(BYTE PacketSequence,RECEIVED_MESSAGE stReceivedMessageBuffer);


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
    /* Function Static variables */
    
    /* Local Variables */
    
    
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
    ConsolePutROMString((ROM char*)"\r\nNDSU ECE 209 Ruisi Modified  ");
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
    /* Function Static variables */
    
    /* Local Variables */
    BYTE PressedButton;
    BYTE bySlaveIndex = 0;
    BYTE byPacketSequence = 0;
    RECEIVED_MESSAGE stReceivedMessage = (RECEIVED_MESSAGE) {0};
    MASTER_STATES_E eMasterStates = INACTIVE;   
    scMainInit();
    while(TRUE)
    {
        switch(eMasterStates)
        {
            case INACTIVE:
                PressedButton = ButtonPressed();
                if (PressedButton == 1)
                {
                     DelayMs(FIVE_SECONDS);       //Allow the end device to CALIBRATE
                    eMasterStates = REQ_ADC_CALC;
                }
                break;
                        
            case REQ_ADC_CALC:       
               scDoGlobalADCRequest();
               DelayMs(ADC_CALC_MAX_TIME_MS);   
               eMasterStates = REQ_SLAVE_RES;
               break;

            case REQ_SLAVE_RES:               
                        for (bySlaveIndex = 0; bySlaveIndex < sizeof(kabySlaves); bySlaveIndex++)
                        {
                            for(byPacketSequence = 0;byPacketSequence < MAX_PACKET_SEQUENCE;byPacketSequence++)
                            {   
                                scReqSlaveStatus(kabySlaves[bySlaveIndex],byPacketSequence );
                                if (scfReceive(&stReceivedMessage))
                                {   
                                    if (stReceivedMessage.Payload[SLAVE_ID_INDEX] == bySlaveIndex)
                                    {
                                        switch ((SLAVE_RES_STATES_E)stReceivedMessage.Payload[COMMAND_INDEX])
                                        {
                                            case SLAVE_NO_ACKNOWLEDGE:
                                                ConsolePutROMString((ROM char *)"ERROR CASE, SLAVE_NO_ACKNOWLEDGE\r\n");
                                                eMasterStates = REQ_ADC_CALC;
                                                break;

                                            case SLAVE_ACKNOWLEDGE:                                                                                
                                                scCollectandSortMessage(stReceivedMessage.Payload[SEQUENCE_INDEX],stReceivedMessage);  
                                                scPrintConsole(bySlaveIndex,byPacketSequence);
                                                break;

                                            default:
                                                ConsolePutROMString((ROM char *)"ERROR CASE, COMMAND_INDEX invalid\r\n");                                                 
                                                break;
                                        }
                                    }
                                }
                            }      
                         }         
                         eMasterStates = REQ_ADC_CALC;
                            break;                                                                                                  
                                                                
            default:
                // Error case
                eMasterStates = REQ_ADC_CALC;
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
    const BYTE kabyDataBuffer[] = { GLOBAL_ID, CALC_ADC_CMD };

    scTransmit((BYTE *)&kabyDataBuffer, sizeof(kabyDataBuffer));
}


/*----------------------------------------------------------------------------
 
@Prototype: static void scReqSlaveStatus(const BYTE kbySlaveID, BYTE byPacketSequence)
 
@Description: Request message response from slave 
@Parameters: const BYTE kbySlaveID input parameter for different slave ID 
 *           BYTE byPacketSequence input parameter for packet sequence
@Returns: void
@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision
05/01/2017       Ruisi Ge           Updated Revision
*----------------------------------------------------------------------------*/
static void scReqSlaveStatus(const BYTE kbySlaveID, BYTE byPacketSequence)
{
    const BYTE kabyDataBuffer[] = { kbySlaveID, REQ_STATUS_CMD, byPacketSequence };
    
    scTransmit((BYTE *)&kabyDataBuffer, sizeof(kabyDataBuffer));
}


static void scReqMissingPack(const BYTE kbySlaveID, BYTE byMissPacketSequence)
{
    const BYTE kabyDataBuffer[] = { kbySlaveID, REQ_STATUS_CMD, byMissPacketSequence };
    
    scTransmit((BYTE *)&kabyDataBuffer, sizeof(kabyDataBuffer)); 
}


static void scCollectandSortMessage(BYTE PacketSequence,RECEIVED_MESSAGE stReceivedMessageBuffer)
{
    BYTE byDataCount;
    byHundredMicroseconds1stByte=stReceivedMessageBuffer.Payload[TIME_1_BYTE];
    byHundredMicroseconds2ndByte=stReceivedMessageBuffer.Payload[TIME_2_BYTE];
    byHundredMicroseconds3rdByte=stReceivedMessageBuffer.Payload[TIME_3_BYTE];
    byHundredMicroseconds4thByte=stReceivedMessageBuffer.Payload[TIME_4_BYTE];
    MaxThreshouldValueHighByte = stReceivedMessageBuffer.Payload[MAX_VALUE_HIGH_BYTE];
    MaxThreshouldValueLowByte=stReceivedMessageBuffer.Payload[MAX_VALUE_LOW_BYTE];
    MaxThreshouldValue= ((unsigned int)MaxThreshouldValueHighByte<<8) + MaxThreshouldValueLowByte;
    TotalHundredMicroseconds = ((unsigned long)byHundredMicroseconds1stByte<<24)+((unsigned long)byHundredMicroseconds2ndByte<<16)+((unsigned int)byHundredMicroseconds3rdByte<<8)+byHundredMicroseconds4thByte;
    
    for (byDataCount=0; byDataCount<MAX_DATA_SIZE;byDataCount++)
        {

            ADCValue[PacketSequence][byDataCount] = stReceivedMessageBuffer.Payload[byDataCount+HEADERCOOMANDSIZE+TIMEHEADERSIZE+THRESHHOLDVALUEHEADER];
 
        }  
}


/*----------------------------------------------------------------------------
 
@Prototype: static void scPrintConsole(BYTE bySlaveID, BYTE byADCValue)
 
@Description: Pretty print some information to console
@Parameters: BYTE bySlaveID 
             RECEIVED_MESSAGE * stReceiveMessageBuffer *  - Out parameter for the received message.
             int DataCount 
@Returns: void
@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision
04/18/2017       Ruisi Ge           updated Revision for multiple data
*----------------------------------------------------------------------------*/
static void scPrintConsole(BYTE bySlaveID,BYTE PacketSequence )
{   
    BYTE byDataCount;   
//    ConsolePutROMString((ROM char*)"\r\nNode ");   
//    ConsolePut(bySlaveID % 10 + '0');
//    ConsolePutROMString((ROM char*)" | ");
//    ConsolePutROMString((ROM char*)"Sequence ");
//    ConsolePut(PacketSequence % 10 + '0');
//    ConsolePutROMString((ROM char*)" | ");
//    ConsolePutROMString((ROM char*)"Start Time ");
//    sprintf(str,"%lu",TotalHundredMicroseconds);
//    ConsolePutROMString((ROM char*)str);
//    ConsolePutROMString((ROM char*)" | ");
//    ConsolePutROMString((ROM char*)"threshold value ");
//    sprintf(str, "%d", MaxThreshouldValue );
//    ConsolePutROMString((ROM char*)str);
//    ConsolePutROMString((ROM char*)" | ADC | ");    
    for (byDataCount =0;byDataCount<MAX_DATA_SIZE;byDataCount++)
    {   
        sprintf(str, "%d", ADCValue[PacketSequence][byDataCount]);
        ConsolePutROMString((ROM char*)str); 
        ConsolePutROMString((ROM char*)",");
    }        
}