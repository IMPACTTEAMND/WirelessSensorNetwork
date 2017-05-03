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
#define MAX_PACKET_SEQUENCE         5
#define MAX_PACKET_SIZE             50         
#define MAX_DATA_PACKET             22          // Max data packet= (MAX_PACKET_SIZE-HEADERCOOMANDSIZE)/2
#define ADC_CALC_MAX_TIME_MS        200
#define SELF_CALIBRATE_MAX_TIME_MS  150
#define RX_TIME                     1
#define HEADERCOOMANDSIZE           3
#define TIMEHEADERSIZE              4

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
};

/* -- TYPEDEFS and STRUCTURES -- */
typedef enum
{
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

 static const BYTE kabySlaves[] = { SLAVE_0_ID, SLAVE_1_ID };
 static int  ADCValue[MAX_PACKET_SEQUENCE][MAX_DATA_PACKET] ;
 static unsigned long TotalHundredMicroseconds; 
 static BYTE MissingPacketSequence;
 static BYTE byHundredMicroseconds1stByte;                 // First Byte for Milliseconds 
 static BYTE byHundredMicroseconds2ndByte;                // Second Byte for Milliseconds
 static BYTE byHundredMicroseconds3rdByte;               //  Third Byte for Milliseconds
 static BYTE byHundredMicroseconds4thByte;              //  Fourth Byte for Milliseconds
 static BYTE byADCHighValue[MAX_PACKET_SEQUENCE][MAX_DATA_PACKET];
 static BYTE byADCLowValue[MAX_PACKET_SEQUENCE][MAX_DATA_PACKET];
 static BYTE byCheckSum[MAX_PACKET_SEQUENCE] ={0};      // this CheckSum is an array, the value could only be 1 or 0, 1 means received, 0 means haven't received 
/* -- STATIC FUNCTION PROTOTYPES -- */
static void scMainInit(void);
static void scTransmit(BYTE *pbyTxBuffer, BYTE byLength);
static void scClearCheckSum(void);
static BOOL scfReceive(RECEIVED_MESSAGE *stReceiveMessageBuffer);
static void scDoGlobalADCRequest(void);
static void scReqSlaveStatus(const BYTE kbySlaveID, BYTE byPacketSequence);
static void scReqMissingPack(const BYTE kbySkaveID, BYTE byMissPacketSequence);
static void scPrintConsole(BYTE bySlaveID);
static void scCollectandSortMessage(BYTE PacketSequence,RECEIVED_MESSAGE stReceivedMessageBuffer);
static BOOL scMessageFullyReceive(void);

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
    /* Function Static variables */
    
    /* Local Variables */
    BYTE bySlaveIndex = 0;
    BYTE byPacketSequence = 0;
    RECEIVED_MESSAGE stReceivedMessage = (RECEIVED_MESSAGE) {0};
    MASTER_STATES_E eMasterStates = REQ_ADC_CALC;   
    REQUEST_RESPOND_STATES_E eRequestRespondStates = REQ_FULL_MESSAGE;
    scMainInit();
    while(TRUE)
    {
        switch(eMasterStates)
        {
            case REQ_ADC_CALC:
//#ifdef DEBUG
//               ConsolePutROMString((ROM char *)"REQ_ADC_CALC\r\n");
//#endif /* ifndef DEBUG */
               
               DelayMs(SELF_CALIBRATE_MAX_TIME_MS);    
               scDoGlobalADCRequest();
               DelayMs(ADC_CALC_MAX_TIME_MS);   
                
               eMasterStates = REQ_SLAVE_RES;
               break;

            case REQ_SLAVE_RES:
                for (bySlaveIndex = 0; bySlaveIndex < sizeof(kabySlaves); bySlaveIndex++)
                {
                     
                    eRequestRespondStates = REQ_FULL_MESSAGE;      //  Whenever start a new device, go to the request full message states
                    switch(eRequestRespondStates)
                    {
                        case REQ_FULL_MESSAGE:
                            for(byPacketSequence = 0;byPacketSequence < MAX_PACKET_SEQUENCE;byPacketSequence++)
                            {   
                                scReqSlaveStatus(kabySlaves[bySlaveIndex],byPacketSequence );
                                if (scfReceive(&stReceivedMessage))
                                {   
            #ifdef DEBUG
                           ConsolePutROMString((ROM char *)"received\r\n");
            #endif /* ifndef DEBUG */
                                    if (stReceivedMessage.Payload[SLAVE_ID_INDEX] == bySlaveIndex)
                                    {
                                        switch ((SLAVE_RES_STATES_E)stReceivedMessage.Payload[COMMAND_INDEX])
                                        {
                                            case SLAVE_NO_ACKNOWLEDGE:
                                                ConsolePutROMString((ROM char *)"ERROR CASE, SLAVE_NO_ACKNOWLEDGE\r\n");
                                                eMasterStates = REQ_ADC_CALC;
                                                break;

                                            case SLAVE_ACKNOWLEDGE:                                    
                                                ConsolePutROMString((ROM char *)"SLAVE_ACKNOWLEDGE\r\n"); 
                                                byCheckSum[stReceivedMessage.Payload[SEQUENCE_INDEX]]=1;          // Fill in the CheckSum Array 
                                                scCollectandSortMessage(stReceivedMessage.Payload[SEQUENCE_INDEX],stReceivedMessage);                                                                   
                                                break;

                                            default:
                                                ConsolePutROMString((ROM char *)"ERROR CASE, COMMAND_INDEX invalid\r\n");
                                                 

                                                break;
                                        }
                                    }
                                    else
                                    {
                                        // Error case
                                        break;
                                    }
                                }
                                else
                                {
                                    // Error case
                                    break;
                                }
                            }
                            eRequestRespondStates= CHECK_MESSAGE;
                            break;
                        case CHECK_MESSAGE:
                            if (scMessageFullyReceive())
                            {
                                /*print the message*/
                                scPrintConsole(bySlaveIndex);
                                scClearCheckSum();                              // clear CheckSum after full receive the message  
                                break;
                            }
                            else
                            {
                                eRequestRespondStates = REQ_MISS_MESSAGE;
                                break;
                            }   
                            break;
                        
                        case REQ_MISS_MESSAGE:
                            scReqMissingPack(kabySlaves[bySlaveIndex],MissingPacketSequence);
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
                                                ConsolePutROMString((ROM char *)"SLAVE_ACKNOWLEDGE\r\n"); 
                                                byCheckSum[stReceivedMessage.Payload[SEQUENCE_INDEX]]=1;          // Fill in the CheckSum Array 
                                                scCollectandSortMessage(stReceivedMessage.Payload[SEQUENCE_INDEX],stReceivedMessage);                                                                   
                                                break;

                                            default:
                                                ConsolePutROMString((ROM char *)"ERROR CASE, COMMAND_INDEX invalid\r\n");                                                 
                                                break;
                                        }
                                    }
                                    else
                                    {
                                        // Error case
                                        break;
                                    }
                                }
                                else
                                {
                                    // Error case
                                    break;
                                }                            
                            eRequestRespondStates = CHECK_MESSAGE;
                            break;
                 
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
    const BYTE kabyDataBuffer[] = { kbySlaveID, REQ_MISS_MESSAGE_CMD, byMissPacketSequence };
    
    scTransmit((BYTE *)&kabyDataBuffer, sizeof(kabyDataBuffer)); 
}


static void scCollectandSortMessage(BYTE PacketSequence,RECEIVED_MESSAGE stReceivedMessageBuffer)
{
    BYTE byDataCount;
    if(PacketSequence == 0)
    {
        byHundredMicroseconds1stByte=stReceivedMessageBuffer.Payload[TIME_1_BYTE];
        byHundredMicroseconds2ndByte=stReceivedMessageBuffer.Payload[TIME_2_BYTE];
        byHundredMicroseconds3rdByte=stReceivedMessageBuffer.Payload[TIME_3_BYTE];
        byHundredMicroseconds4thByte=stReceivedMessageBuffer.Payload[TIME_4_BYTE];
        TotalHundredMicroseconds = ((unsigned long)byHundredMicroseconds1stByte<<24)+((unsigned long)byHundredMicroseconds2ndByte<<16)+((unsigned int)byHundredMicroseconds3rdByte<<8)+byHundredMicroseconds4thByte;
        for (byDataCount=0; byDataCount<MAX_DATA_PACKET;byDataCount++)
        {
            byADCHighValue[PacketSequence][byDataCount]=stReceivedMessageBuffer.Payload[2*byDataCount+HEADERCOOMANDSIZE+TIMEHEADERSIZE];
            byADCHighValue[PacketSequence][byDataCount]=stReceivedMessageBuffer.Payload[2*byDataCount+HEADERCOOMANDSIZE+TIMEHEADERSIZE+1];
            ADCValue[PacketSequence][byDataCount]= ((unsigned int) byADCHighValue[byDataCount] << 8) + byADCLowValue[byDataCount];     
        }
    }
    else
    {
        for (byDataCount=0; byDataCount<MAX_DATA_PACKET;byDataCount++)
        {
            byADCHighValue[PacketSequence][byDataCount]=stReceivedMessageBuffer.Payload[2*byDataCount+HEADERCOOMANDSIZE];
            byADCHighValue[PacketSequence][byDataCount]=stReceivedMessageBuffer.Payload[2*byDataCount+HEADERCOOMANDSIZE+1];
            ADCValue[PacketSequence][byDataCount]= ((unsigned int) byADCHighValue[byDataCount] << 8) + byADCLowValue[byDataCount];     
        }
    }
}

static void scClearCheckSum(void)
{
    BYTE byPacketNumber;
    for(byPacketNumber=0; byPacketNumber< MAX_PACKET_SEQUENCE;byPacketNumber++)
        {
            byCheckSum[byPacketNumber] = 0;
        }                                         // Whenever start in th REQ_ADC_CALC case, initiate the checksum array
}

static BOOL scMessageFullyReceive(void)
{
    BYTE byPacketSequence;
    BOOL MessageFullyReceive= TRUE;
    for (byPacketSequence=0;byPacketSequence<MAX_PACKET_SEQUENCE;byPacketSequence++)
    {
        if (byCheckSum[byPacketSequence]==0)
        {
            MessageFullyReceive = FALSE;
            MissingPacketSequence=byPacketSequence;
        }
    }
    return MessageFullyReceive;
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
static void scPrintConsole(BYTE bySlaveID)
{   
    BYTE byDataCount;
    BYTE PacketSequence;
    char str[100];
    ConsolePutROMString((ROM char*)"Node ");   
    ConsolePut(bySlaveID % 10 + '0');
    ConsolePutROMString((ROM char*)" | ");
    ConsolePutROMString((ROM char*)"Sequence ");
    ConsolePutROMString((ROM char*)"Start Time ");
    sprintf(str,"%lu",TotalHundredMicroseconds);
    ConsolePutROMString((ROM char*)str);
    ConsolePutROMString((ROM char*)" | ");
    for (PacketSequence = 0;PacketSequence<MAX_PACKET_SEQUENCE;PacketSequence++)
    {
        for (byDataCount =0;byDataCount<MAX_DATA_PACKET;byDataCount++)
        {
            ConsolePutROMString((ROM char*)"Value  ");
            sprintf(str, "%d", ADCValue[PacketSequence][byDataCount]);
            ConsolePutROMString((ROM char*)str); 
            ConsolePutROMString((ROM char*)" | ");
        }
    }
    
}

