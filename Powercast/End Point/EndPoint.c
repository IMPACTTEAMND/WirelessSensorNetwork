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
#define VBG_VAL         1228800UL   // VBG = 1.2V, VBG_VAL = 1200 mV * 1024 counts
#define T_BIAS          10000.0     // Bias res for temp sensor
#define B_CONST         3380.0      // B Constant of thermistor
#define R_0             10000.0     // Thermistor resistance at 25C
#define T_0             298.15      // Temp in kelvin at 25C
#define MYCHANNEL 25
#define CODE_VERSION 12
#define EXTERNAL_SENSOR_CHANNEL 2

#define RX_TIME 2
#define SAMPLE_ADC_VALUE 8
#define MAX_PACKET_SIZE 20
#define UNDEFINED 0
#define FIVE_SECONDS 50000
#define THIRTY_SECONDS 300000

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


typedef enum
{
    READ_ADC_CMD,
    REQ_STATUS_CMD
} COMMANDS_E;


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
    READ_ADC,
    SLAVE_RES,
    SENSOR_CALIBRATION
} SLAVE_STATES_E;

typedef enum
{
    SLAVE_NO_ACKNOWLEDGE,
    SLAVE_ACKNOWLEDGE
} SLAVE_RES_STATES_E;

typedef enum
{
    CALIBRATION_INIT,
    CALIBRATE,
    CALIBRATION_DONE     
} CALIBRATION_STATES_E;


/* -- STATIC AND GLOBAL VARIABLES -- */
/* This value indicates that the last command
   received from master was processed successfully */
static BOOL scfSlaveStatus;
static BYTE scabyResponseBuffer[MAX_PACKET_SIZE];
static DWORD scdwHundredMicroSecondsADC;
static DWORD scdwHundredMicroSecondsCalibration;
static WORD scawCalibrationRunningAvgValues[10];
// We will not overflow since we will have only 10
static WORD scwCalibrationMaxThreshold;
static WORD scwCalibrationRunningAvg;


/* -- STATIC FUNCTION PROTOTYPES -- */
static void scMainInit(void);
static void scTimerInterruptInit (void);
static void scTransmit(BYTE *pbyTxBuffer, BYTE byLength);
static BOOL scfReceive(RECEIVED_MESSAGE *stReceiveMessageBuffer);
static WORD scwADCRead(WORD wChannel);
static void scCalibrateSensor(WORD wSensorChannel);
static BOOL scfDoReadADC(WORD wSensorChannel);
static SLAVE_STATES_E sceConvertCommandToState(COMMANDS_E eCommand);


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
    scTimerInterruptInit();
    
    SPI1STAT = 0x8000;  // Enable SPI bus to talk radio
    
    /*******************************************************************/
    // Function MiApp_ProtocolInit initialize the protocol stack. The
    // only input parameter indicates if previous network configuration
    // should be restored. In this simple example, we assume that the
    // network starts from scratch.
    /*******************************************************************/
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
    scfSlaveStatus = SLAVE_NO_ACKNOWLEDGE;
    scdwHundredMicroSecondsADC = UNDEFINED;
    scdwHundredMicroSecondsCalibration = UNDEFINED;
    scwCalibrationMaxThreshold = UNDEFINED;
    scwCalibrationRunningAvg = UNDEFINED;
    
    for (byI = 0; byI < (sizeof(scawCalibrationRunningAvgValues)/sizeof(scawCalibrationRunningAvgValues[0])); byI++)
    {
        scawCalibrationRunningAvgValues[byI] = UNDEFINED;
    }
    
    for (byI = 0; byI < (sizeof(scabyResponseBuffer)/sizeof(scabyResponseBuffer[0])); byI++)
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
    CALIBRATION_STATES_E eCalibrationStates = CALIBRATION_INIT;
    BOOL fCalibrating = TRUE;
    
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
                else if (fCalibrating)
                {
                    eSlaveStates = SENSOR_CALIBRATION;
                }
                break;

            case SENSOR_CALIBRATION:
                switch (eCalibrationStates)
                {
                    case CALIBRATION_INIT:
                        // Reset calibration timer
                        scdwHundredMicroSecondsCalibration = 0;
                        eCalibrationStates = CALIBRATE;
                        break;
                        
                    case CALIBRATE:
                        // Calibrate for 5 seconds
                        if (scdwHundredMicroSecondsCalibration < FIVE_SECONDS)
                        {
                            scCalibrateSensor(EXTERNAL_SENSOR_CHANNEL);
                        }
                        else
                        {
                            eCalibrationStates = CALIBRATION_DONE;
                        }
                        break;
                        
                    case CALIBRATION_DONE:
                        fCalibrating = FALSE;
                        break;
                        
                    default:
                        // We should never get here
                        eCalibrationStates = CALIBRATION_INIT;
                        break;
                }
                
                eSlaveStates = INACTIVE;
                break;
                
            case READ_ADC:
                if (scfDoReadADC(EXTERNAL_SENSOR_CHANNEL))
                {
                    scfSlaveStatus = SLAVE_ACKNOWLEDGE;
                }
                else
                {
                    scfSlaveStatus = SLAVE_NO_ACKNOWLEDGE;
                }
                eSlaveStates = INACTIVE;
                break;

            case SLAVE_RES: //TODO: more work needed to make generic
                scabyResponseBuffer[SLAVE_ID_INDEX] = UNIQUE_SLAVE;
                scabyResponseBuffer[COMMAND_INDEX] = (BYTE)scfSlaveStatus;
                if (scfSlaveStatus == SLAVE_ACKNOWLEDGE)
                {
                    scTransmit((BYTE *)&scabyResponseBuffer, MAX_PACKET_SIZE);
                }
                else
                {
                    scTransmit((BYTE *)&scabyResponseBuffer, 2);
                }
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
 
@Prototype: static BOOL scfDoReadADC(WORD wSensorChannel)
 
@Description: Read as many ADC values as allowed after conditions to read are satisfied

@Parameters: WORD wSensorChannel - The sensor to read

@Returns: BOOL - TRUE on Success, FALSE on Failure

@Revision History:
DATE             NAME               REVISION COMMENT
05/17/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static BOOL scfDoReadADC(WORD wSensorChannel)
{
    BOOL fRetVal = FALSE;
    WORD wADCValue = 0;
    WORD wPacketIndex = ADC_VALUE_INDEX;
    
    // Reset ADC Timer
    scdwHundredMicroSecondsADC = 0; 
    
    while (scdwHundredMicroSecondsADC < THIRTY_SECONDS)
    {
        wADCValue = scwADCRead(wSensorChannel);
        
        if (wADCValue > scwCalibrationMaxThreshold)
        {
            while (wPacketIndex < MAX_PACKET_SIZE)
            {
                scabyResponseBuffer[wPacketIndex] = (BYTE)((wADCValue - scwCalibrationRunningAvg) >> 1);
                wPacketIndex++;
                wADCValue = scwADCRead(wSensorChannel);
            }
            break;
        }
    }
    
    fRetVal = (scdwHundredMicroSecondsADC < THIRTY_SECONDS);
    
    return fRetVal;
}


/*----------------------------------------------------------------------------
 
@Prototype: static void scCalibrateSensor(void)
 
@Description: Calculate running average and max threshold for sensor calibration

@Parameters: WORD wSensorChannel - The sensor to calibrate

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
05/17/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scCalibrateSensor(WORD wSensorChannel)
{
    // Used for running average calculation
    static BYTE smbyCurrentCount = 0;
    static BOOL smfOnInitial = TRUE;
    
    WORD wPresentADCValue = scwADCRead(wSensorChannel);
    WORD wSum = 0;
    BYTE byI;
    
    // On initial values, to populate the array of running avg
    if ((smbyCurrentCount < (sizeof(scawCalibrationRunningAvgValues)/sizeof(scawCalibrationRunningAvgValues[0]))) && smfOnInitial) 
    {
        scawCalibrationRunningAvgValues[smbyCurrentCount] = wPresentADCValue;
        smbyCurrentCount++;
    }
    // Reset count to over write oldest values
    else if (smbyCurrentCount >= (sizeof(scawCalibrationRunningAvgValues)/sizeof(scawCalibrationRunningAvgValues[0])))
    {
        smfOnInitial = FALSE;
        smbyCurrentCount = 0;
    }
    // On values after initial values
    else
    {
        scawCalibrationRunningAvgValues[smbyCurrentCount] = wPresentADCValue;
        smbyCurrentCount++;
        
        // Sum all the values in the array
        for (byI = 0; byI < (sizeof(scawCalibrationRunningAvgValues)/sizeof(scawCalibrationRunningAvgValues[0])); byI++)
        {
            wSum += scawCalibrationRunningAvgValues[byI];
        }
        
        // Calculate the running average
        scwCalibrationRunningAvg = (wSum / (sizeof(scawCalibrationRunningAvgValues)/sizeof(scawCalibrationRunningAvgValues[0])));
    }
    
    if (wPresentADCValue > scwCalibrationMaxThreshold)
    {
        scwCalibrationMaxThreshold = wPresentADCValue;
    }
}


/*----------------------------------------------------------------------------
 
@Prototype: static SLAVE_STATES_E sceConvertCommandToState(const COMMANDS_E keCommand)
 
@Description: Convert a received command from master to a slave state

@Parameters: COMMANDS_E eCommand - The command from master

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/13/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static SLAVE_STATES_E sceConvertCommandToState(COMMANDS_E eCommand)
{
    SLAVE_STATES_E eSlaveState = INACTIVE;

    switch (eCommand)
    {
        case READ_ADC_CMD:
            eSlaveState = READ_ADC;
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
    
    DelayMs(100);
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
    BYTE byTimeout = 0xFF;

    // Timeout after 255ms
    while (byTimeout > 0)
    {
        byTimeout--;
        if (MiApp_MessageAvailable())
        {
            * stReceiveMessageBuffer = rxMessage;
            MiApp_DiscardMessage();
            fRetVal = TRUE;
            break;
        }
        DelayMs(RX_TIME);
    }
    
    return fRetVal;
}


/*----------------------------------------------------------------------------
 
@Prototype: static WORD scwADCRead(WORD wADCChannel)
 
@Description: Read an ADC value from a given ADC channel

@Parameters: WORD wADCChannel - ADC Channel 
 
@Returns: WORD: The ADC value
           

@Revision History:
DATE             NAME               REVISION COMMENT
04/12/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static WORD scwADCRead(WORD wADCChannel)
{
    WORD wI = 0;
    WORD wADCVal = SAMPLE_ADC_VALUE;
    
    SPI1STAT = 0x0000;  // Disable SPI bus to read ADC
        
    AD1CHS = wADCChannel;           // set channel to measure 
    AD1CON1bits.ADON = 1;        // turn ADC on for taking readings
    for (wI = 0; wI < 150; wI++);
    AD1CON1bits.SAMP = 1;       // start sampling
    while (!AD1CON1bits.DONE);  // wait for ADC to complete
    wADCVal = ADC1BUF0;
    AD1CON1bits.ADON = 0;       // turn ADC off for before taking next reading
    
    SPI1STAT = 0x8000;  // Enable SPI bus to talk radio

    return wADCVal;
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
    T1CON = 0x0000; 		// stops the timer1 and reset control flag
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
*					after 100 us, updates total time 
*
* Note:			    
**********************************************************************/
void _ISRFAST __attribute__((interrupt, auto_psv)) _T1Interrupt(void) 
{
    scdwHundredMicroSecondsADC++;
    scdwHundredMicroSecondsCalibration++;
   	IFS0bits.T1IF = 0;						// Clear Timer 1 interrupt flag
	TMR1 = 0xFFCD;                          //0XFFFF - 0x0050=50 pulses = 	100 usec count at Fosc = 8MHZ with Timer prescalar of 1:8; 
	return;
}