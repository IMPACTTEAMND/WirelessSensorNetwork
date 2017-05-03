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
#define MYCHANNEL           25
#define EXT_CHANNEL         2  
#define SAMPLE_ADC_VALUE    5
#define SELF_CALIBRATE_MAX_TIME_MS    150
#define MAX_PACKET_SIZE     50          // Maxmium Size in one packet
#define MAX_DATA_SIZE       22          // Maxmium ADC Data Size =  (MAX_PACKET_SIZE-HEADERCOOMANDSIZE)/2
#define MAX_PACKET_SEQUENCE 5 
#define UNDEFINED           0
#define HEADERCOOMANDSIZE   3         // header include 1 BYTE DEVICE ID,1 BYTE COMMAND, 1 BYTE SEQUENCE, 4 BYTE TIME  
#define TIMEHEADERSIZE      4         // Timer header include 4 byte timer header size

/* Uncomment for debugging */
//#define DEBUG
enum
{
    SLAVE_0_ID,
    SLAVE_1_ID,
    SLAVE_2_ID,
    SLAVE_3_ID,
    GLOBAL_ID = 0xFF
};
//TODO: EDIT THIS FOR UNIQUE SLAVE DEVICE
#define UNIQUE_SLAVE SLAVE_0_ID

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
    INACTIVE,
    SELF_CALIBRATE,
    ADC_CALC,
    SLAVE_RES,
} SLAVE_STATES_E;

typedef enum
{
    SLAVE_NO_ACKNOWLEDGE,
    SLAVE_ACKNOWLEDGE
} SLAVE_RES_STATES_E;

typedef enum
{
    TIMER_INITIATE,
    ADC_INITIATE,
    ADC_MEASURING,
}ADC_STATES_E;

typedef enum
{
    CALC_ADC_CMD,
    REQ_STATUS_CMD,
    REQ_MISS_MESSAGE_CMD
} COMMANDS_E;

/* -- STATIC AND GLOBAL VARIABLES -- */
static unsigned int  scMaxThreshhold=0;
static uint32_t scHundredMicroseconds;
static unsigned char scHundredMicroseconds1stByte;      // First Byte for Milliseconds 
static unsigned char scHundredMicroseconds2ndByte;       // Second Byte for Milliseconds
static unsigned char scHundredMicroseconds3rdByte;
static unsigned char scHundredMicroseconds4thByte; 
static unsigned int  scADCValue[MAX_PACKET_SEQUENCE][MAX_DATA_SIZE]; 
static unsigned char scabyResponseBuffer[MAX_PACKET_SIZE];
static unsigned char scADCH[MAX_PACKET_SEQUENCE][MAX_DATA_SIZE];
static unsigned char scADCL[MAX_PACKET_SEQUENCE][MAX_DATA_SIZE];
static uint32_t scADCStartTime;
/* -- STATIC FUNCTION PROTOTYPES -- */
static void scMainInit(void);
static void scTransmit(BYTE *pbyTxBuffer, BYTE byLength);
static BOOL scfReceive(RECEIVED_MESSAGE *stReceiveMessageBuffer);
static unsigned int  scADCRead(WORD wChannel);
static void scSpiltData(void);
static void scAllocateRespondBuffer(BYTE PacketSequence);
static int scSelfCalibratingMax(WORD wADCChannel,BYTE CalibrateTimeMS );
static void TimerInitiate (void);
void _ISR _DefaultInterrupt(void);	// interrupt for Timer1, used for keeping time

/* This value indicates that the last command
   received from master was processed successfully */
static BOOL scfSlaveStatus;
static SLAVE_STATES_E sceConvertCommandToState(const COMMANDS_E keCommand);


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
    BYTE byI;

    BoardInit();
    
    /*******************************************************************/
    // Function MiApp_ProtocolInit initialize the protocol stack. The
    // only input parameter indicates if previous network configuration
    // should be restored. In this simple example, we assume that the
    // network starts from scratch.
    /*******************************************************************/
        
    SPI1STAT = 0x8000;  // Enable SPI bus to talk radio
    MiApp_ProtocolInit(FALSE);
    // Set default channel
    MiApp_SetChannel(MYCHANNEL);

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

    /* Initialize static variables */
    scADCValue[MAX_PACKET_SEQUENCE][MAX_DATA_SIZE] = 0 ;
    scfSlaveStatus = SLAVE_NO_ACKNOWLEDGE;

    for (byI = 0; byI < sizeof(scabyResponseBuffer); byI++)
    {
        scabyResponseBuffer[byI] = UNDEFINED;
    }
}


/*----------------------------------------------------------------------------

@Description: The  Application main entry point

@Parameters: void

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/12/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
int main(void)
{
    RECEIVED_MESSAGE stReceivedMessage = (RECEIVED_MESSAGE) {0};
    SLAVE_STATES_E eSlaveStates = INACTIVE;
    ADC_STATES_E eADCStates = TIMER_INITIATE;
    BYTE byDataCount ;
    BYTE byPacketSequence ;
    scMainInit();
    
    while(TRUE)
    {
        switch(eSlaveStates)
        {
            case INACTIVE:        
                if (scfReceive((RECEIVED_MESSAGE *)&stReceivedMessage))
                {
                    if ((stReceivedMessage.Payload[SLAVE_ID_INDEX] == GLOBAL_ID) ||
                        (stReceivedMessage.Payload[SLAVE_ID_INDEX] == UNIQUE_SLAVE))
                    {
                        eSlaveStates = sceConvertCommandToState((COMMANDS_E)stReceivedMessage.Payload[COMMAND_INDEX]);
                    }
                    else
                    {
                        stReceivedMessage = (RECEIVED_MESSAGE) {0};
                    }
                }
                else
                {
                    eSlaveStates = SELF_CALIBRATE;
                    break;
                }
                break;
                
            case SELF_CALIBRATE:
                scMaxThreshhold= scSelfCalibratingMax(EXT_CHANNEL,SELF_CALIBRATE_MAX_TIME_MS);       
                eSlaveStates = INACTIVE;      
                break;

            case ADC_CALC:
                switch(eADCStates)
                {
                    case TIMER_INITIATE:
                        TimerInitiate();
                        eADCStates = ADC_INITIATE;
                        break;
                    case ADC_INITIATE:
                        scADCValue[0][0]=scADCRead(EXT_CHANNEL);           
                        scADCStartTime = scHundredMicroseconds;            // ADC Start recording time 
                        if(scADCValue[0][0]>scMaxThreshhold)
                            {
                               eADCStates = ADC_MEASURING;
                            }
                        else if(scHundredMicroseconds>600000)             // when the time is bigger than 60 seconds
                            {
                                eSlaveStates = INACTIVE;                // Time out if wait longer than 60 seconds
                            }
                        break;
                    case ADC_MEASURING:
                        for(byPacketSequence = 0;byPacketSequence<MAX_PACKET_SEQUENCE;byPacketSequence++)
                        {
                            for(byDataCount =0;byDataCount<MAX_DATA_SIZE;byDataCount++)
                            {
                                scADCValue[byPacketSequence][byDataCount] = scADCRead(EXT_CHANNEL);
                            }
                        }
                        scSpiltData();                       
                        scfSlaveStatus = SLAVE_ACKNOWLEDGE;
                        eSlaveStates = INACTIVE;             
                        break;
                }        
                break;
            case SLAVE_RES: //TODO: more work needed to make generic
                scAllocateRespondBuffer(stReceivedMessage.Payload[SLAVE_ID_INDEX]);
                scTransmit((BYTE *)&scabyResponseBuffer, MAX_PACKET_SIZE);
                eSlaveStates = INACTIVE;
                break;
                
            default:
                // Error case
                eSlaveStates = INACTIVE;
                break;
        }
    }
    return 0;
}


/*----------------------------------------------------------------------------
 
@Prototype: static SLAVE_STATES_E eConvertCommandToState(const BYTE kbyCommand)
 
@Description: Convert a received command from master to a slave state

@Parameters: const COMMANDS_E keCommand - The command from master

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/13/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static SLAVE_STATES_E sceConvertCommandToState(const COMMANDS_E keCommand)
{
    SLAVE_STATES_E eSlaveState = INACTIVE;

    switch (keCommand)
    {
        case CALC_ADC_CMD:
            eSlaveState = ADC_CALC;
            break;

        case REQ_STATUS_CMD:
            eSlaveState = SLAVE_RES;
            break;
                        
        default:
            // Error case
            eSlaveState = INACTIVE;
            break;
    }

    return eSlaveState;
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
 
@Description: Wait until we receive a packet

@Parameters: RECEIVED_MESSAGE * stReceiveMessageBuffer - Out parameter for the 
             received message. 
 
@Returns: TRUE: If we received a packet, FALSE otherwise
           

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static BOOL scfReceive(RECEIVED_MESSAGE * stReceiveMessageBuffer)
{
    BOOL fRetVal = FALSE;

    if (MiApp_MessageAvailable())
    {
        * stReceiveMessageBuffer = rxMessage;
        MiApp_DiscardMessage();

        fRetVal = TRUE;
    }

    return fRetVal;
}


/*----------------------------------------------------------------------------
 
@Prototype: static int scu32ADCRead(WORD wADCChannel)
 
@Description: Read an ADC value from a given ADC channel

@Parameters: WORD wADCChannel - ADC Channel 
 
@Returns: int: The ADC value - the microcontroller has 10 bit adc, hence byte is not correct type 
           

@Revision History:
DATE             NAME               REVISION COMMENT
04/12/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static unsigned int scADCRead(WORD wADCChannel)
{
    WORD wI = 0;
    unsigned int LOCAL_ADCVal;
//    BYTE byADCVal = SAMPLE_ADC_VALUE;
#if 0
    AD1CHS = wADCChannel;           // set channel to measure 
    AD1CON1bits.ADON = 1;        // turn ADC on for taking readings
    for (wI = 0; wI < 150; wI++);
    AD1CON1bits.SAMP = 1;       // start sampling
    while (!AD1CON1bits.DONE);  // wait for ADC to complete
    LOCAL_ADCVal = ADC1BUF0;
    AD1CON1bits.ADON = 0;       // turn ADC off for before taking next reading
#endif

    /* TODO: GET RID OF THIS, FOR SIMULATION PURPOSES ONLY */
    (void)wADCChannel;
     LOCAL_ADCVal = SAMPLE_ADC_VALUE;
    DelayMs(1);
//    for(wI = 0; wI < 150; wI++)                                     // delay around 100 us 
   
//      Delay10us(100);
    return LOCAL_ADCVal;
}
/*----------------------------------------------------------------------------
 
@Prototype: static void scSpiltData(void)
 
@Description: Spilt the ADC and time data to bytes

@Parameters: None 
 
@Returns: None
           

@Revision History:
DATE             NAME               REVISION COMMENT
04/18/2017       Ruisi Ge           Initial Revision

*----------------------------------------------------------------------------*/
static void scSpiltData(void)
{
    BYTE bySpiltDataCount;
    BYTE byPacketSequence;
    for (byPacketSequence;byPacketSequence<MAX_PACKET_SEQUENCE;byPacketSequence++)
    {
        for (bySpiltDataCount=0;bySpiltDataCount<MAX_DATA_SIZE;bySpiltDataCount++)
        {
            scADCH[byPacketSequence][bySpiltDataCount] = scADCValue[byPacketSequence][bySpiltDataCount] >> 8;
            scADCL[byPacketSequence][bySpiltDataCount] = scADCValue[byPacketSequence][bySpiltDataCount]; 
        }
    }
   scHundredMicroseconds1stByte=scADCStartTime>>24;
   scHundredMicroseconds2ndByte=scADCStartTime>>16;
   scHundredMicroseconds3rdByte=scADCStartTime>>8;
   scHundredMicroseconds4thByte=scADCStartTime;
}


/*----------------------------------------------------------------------------
 
@Prototype: static void scAllocateRespondBuffer(void)
 
@Description: allocate the ADC data to RespondBuffer

@Parameters: BYTE, Packet Number 
 
@Returns: None
           

@Revision History:
DATE             NAME               REVISION COMMENT
04/18/2017       Ruisi Ge           Initial Revision

*----------------------------------------------------------------------------*/
static void scAllocateRespondBuffer(BYTE PacketSequence)
{
    int DataCount;
    scabyResponseBuffer[SLAVE_ID_INDEX] = UNIQUE_SLAVE;
    scabyResponseBuffer[COMMAND_INDEX] = (BYTE)scfSlaveStatus;
    scabyResponseBuffer[SEQUENCE_INDEX] = PacketSequence;
    if (PacketSequence = 0)             // Packet zero contains four byte time header
    {
    scabyResponseBuffer[TIME_1_BYTE] = scHundredMicroseconds1stByte;
    scabyResponseBuffer[TIME_2_BYTE] = scHundredMicroseconds2ndByte;
    scabyResponseBuffer[TIME_3_BYTE] = scHundredMicroseconds3rdByte;
    scabyResponseBuffer[TIME_4_BYTE] = scHundredMicroseconds4thByte;
    for (DataCount=0;DataCount< MAX_DATA_SIZE;DataCount++)
        {
            scabyResponseBuffer[2*DataCount+HEADERCOOMANDSIZE+TIMEHEADERSIZE] = scADCH[PacketSequence][DataCount];
            scabyResponseBuffer[2*DataCount+HEADERCOOMANDSIZE+TIMEHEADERSIZE+1] = scADCL[PacketSequence][DataCount];
        }
    }
    else
    {
        for(DataCount=0;DataCount< MAX_DATA_SIZE;DataCount++)
        {
            scabyResponseBuffer[2*DataCount+HEADERCOOMANDSIZE] = scADCH[PacketSequence][DataCount];
            scabyResponseBuffer[2*DataCount+HEADERCOOMANDSIZE+1] = scADCL[PacketSequence][DataCount];
        }
    }
    
}


/*----------------------------------------------------------------------------
 
@Prototype: static int scSelfCalibratingMax(WORD wADCChannel,BYTE CalibrateTimeMS )
 
@Description: find the max value from SelfCalibratingMax

@Parameters: WORD wADCChannel  - ADC Channel, 
 *           BYTE CalibrateTimeMS  - Milliseconds in time to self calibrate
 
@Returns: None
           

@Revision History:
DATE             NAME               REVISION COMMENT
04/18/2017       Ruisi Ge           Initial Revision

*----------------------------------------------------------------------------*/
static int scSelfCalibratingMax(WORD wADCChannel,BYTE CalibrateTimeMS )
{
    /*Work to do, this function for now is use to simulate the process,need to work later*/
    int scSelfCalibratingMax =2;
    (void)wADCChannel;  
    DelayMs(CalibrateTimeMS);
    
    return scSelfCalibratingMax;

}

/*----------------------------------------------------------------------------
 
@Prototype: static void TimerInitiate (void)
 
@Description: initiate the timer 

@Parameters: None 
 
@Returns: None
           

@Revision History:
DATE             NAME               REVISION COMMENT
04/18/2017       Ruisi Ge           Initial Revision

*----------------------------------------------------------------------------*/
static void TimerInitiate (void)
{
    scHundredMicroseconds = 0;
    T1CON = 0x0000; 		// stops the timer1 and reset control flag
//  	TMR1 = 0x3CAF;       	// 0xFFFF - 0xC350 = 50,000 pulses = 100 msec count at Fosc=8MHz with Timer prescalar of 1:8  
    TMR1 = 0xFFCD;                          //0XFFFF - 0x0050=50 pulses = 	100 usec count at Fosc = 8MHZ with Timer prescalar of 1:8
  	IPC0bits.T1IP =0x3; 	// setup Timer1 interrupt for desired priority level
	IEC0bits.T1IE = 1; 		// enable Timer1 interrupts	
	T1CON = 0x8010;		 	// enable timer1 with prescalar of 1:8 
}

/*********************************************************************
* Function:         void T1Interrupt(void)
*
* PreCondition:     none
*
* Input:		    none
*
* Output:		    none
*
* Side Effects:	    none
*
* Overview:		    Interrupt function for Timer1.  Set up to time out
*					after 10 us, updates total time 
*
* Note:			    
**********************************************************************/
void _ISRFAST __attribute__((interrupt, auto_psv)) _T1Interrupt(void) 
{
    scHundredMicroseconds++;
   	IFS0bits.T1IF = 0;						// Clear Timer 1 interrupt flag
	TMR1 = 0xFFCD;                          //0XFFFF - 0x0050=50 pulses = 	100 usec count at Fosc = 8MHZ with Timer prescalar of 1:8; 
	return;
}