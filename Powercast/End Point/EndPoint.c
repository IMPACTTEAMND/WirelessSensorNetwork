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
//TODO: EDIT THIS FOR UNIQUE SLAVE DEVICE
#define UNIQUE_SLAVE SLAVE_0_ID

/* -- GLOBAL VARIABLES -- */
volatile WORD_BYTE_U gvuwTicks;
volatile WORD_BYTE_U gvuwTimer;
volatile WORD_BYTE_U gvuwPositionTicks;

// We will not overflow since we will have only 10 bits in each of 10 ADCs. 1024*10=~11000
volatile WORD_BYTE_U gvuwCalibrationMaxThreshold;
volatile WORD_BYTE_U gvuwCalibrationMinThreshold;
volatile WORD_BYTE_U gvuwCalibrationSampleAvg;

/* -- STATIC VARIABLES -- */
static RECEIVED_MESSAGE scstReceivedMessage;
static BYTE scbySlaveStatus;
static BYTE scaabyResponseBuffer[TOTAL_RESPONSE_BUFFERS][MAX_PACKET_SIZE];
static BYTE scabyPositionTimerBuffer[POSITION_TIMER_BUFFER_SIZE];

/* -- STATIC FUNCTION PROTOTYPES -- */
static void scMainInit(void);
static void scTimerInterruptInit (void);
static void scTransmit(BYTE * pbyBuffer, BYTE byLength);
static BOOL scfReceive(RECEIVED_MESSAGE *stReceiveMessageBuffer);
static WORD scwADCRead(WORD wADCChannel, WORD wDelay);
static void scDoCalibrateSensor(WORD wSensorChannel);
static BOOL scfDoReadADC(WORD wSensorChannel);
static void scDoMeasurePosition(WORD wADCChannel);


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
* Overview:         Interrupt function for Timer1
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
04/18/2017       Ali Haidous           Initial Revision
*----------------------------------------------------------------------------*/
static void scTimerInterruptInit (void)
{
	// Set up Timer1
  	T1CON = 0x0000; 		// stops the timer1 and reset control flag
  	TMR1 = PULSES;
  	IPC0bits.T1IP = HIGHEST_PRIORITY; 	// setup Timer1 interrupt for desired priority level
	IEC0bits.T1IE = 1; 		// enable Timer1 interrupts	
	T1CON = 0x8010;		 	// enable timer1 with prescalar of 1:8 
    
    gvuwTimer.wOrd = 0;
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
    BYTE byI;
    BYTE byJ;

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

    /* Initialize static variables */
    scstReceivedMessage = (RECEIVED_MESSAGE) {0};
    scbySlaveStatus = INVALID_STATUS;
    gvuwTicks.wOrd = 0;
    gvuwTimer.wOrd = 0;
    gvuwPositionTicks.wOrd = 0;
    
    gvuwCalibrationMaxThreshold.wOrd = 0;
    gvuwCalibrationMinThreshold.wOrd = 0xFFFF;
    gvuwCalibrationSampleAvg.wOrd = 0;

    for (byI = 0; byI < TOTAL_RESPONSE_BUFFERS; byI++)
    {
        for (byJ = 0; byJ < MAX_PACKET_SIZE; byJ++)
        {
            scaabyResponseBuffer[byI][byJ] = 0;
        }
    }
    
    for (byI = 0; byI < TOTAL_RESPONSE_BUFFERS; byI++)
    {
        scaabyResponseBuffer[byI][SLAVE_ID_INDEX] = UNIQUE_SLAVE;
        scaabyResponseBuffer[byI][STATUS_INDEX] = scbySlaveStatus;
        scaabyResponseBuffer[byI][BUFFER_INDEX] = byI;
    }
    
    for (byI = 0; byI < POSITION_TIMER_BUFFER_SIZE; byI++)
    {
        scabyPositionTimerBuffer[byI] = 0;
    }
    
    scabyPositionTimerBuffer[SLAVE_INDEX] = UNIQUE_SLAVE;
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
    COMMANDS_E eSlaveCommand = INVALID_CMD;
    BYTE byBufferIndex = 0;
    
    scMainInit();
    
    while(TRUE)
    {
        /* This function maintains the operation of the stack
           It should be called as often as possible. */
        P2PTasks();

        switch(eSlaveCommand)
        {
            case DO_CALIBRATION_CMD:
                scDoCalibrateSensor(ANALOG_CHANNEL);
                eSlaveCommand = INVALID_CMD;
                break;
            
            case DO_MEASURE_POSITION_CMD:
                scDoMeasurePosition(ANALOG_CHANNEL);
                eSlaveCommand = INVALID_CMD;
                break;
                
            case DO_READ_ADC_CMD:
                if (scfDoReadADC(ANALOG_CHANNEL))
                {
                    scbySlaveStatus = READ_ADC_PASSED;
                }
                else
                {
                    scbySlaveStatus = READ_ADC_FAILED;
                }
                
                for (byBufferIndex = 0; byBufferIndex < TOTAL_RESPONSE_BUFFERS; byBufferIndex++)
                {
                    scaabyResponseBuffer[byBufferIndex][STATUS_INDEX] = scbySlaveStatus;
                }

                eSlaveCommand = INVALID_CMD;
                break;

            case REQ_BUFFER_CMD:
                byBufferIndex = scstReceivedMessage.Payload[BUFFER_INDEX];
                scaabyResponseBuffer[byBufferIndex][MAX_THRESHOLD_INDEX_H] = gvuwCalibrationMaxThreshold.abyTe[1];
                scaabyResponseBuffer[byBufferIndex][MAX_THRESHOLD_INDEX_L] = gvuwCalibrationMaxThreshold.abyTe[0];
                scaabyResponseBuffer[byBufferIndex][MIN_THRESHOLD_INDEX_H] = gvuwCalibrationMinThreshold.abyTe[1];
                scaabyResponseBuffer[byBufferIndex][MIN_THRESHOLD_INDEX_L] = gvuwCalibrationMinThreshold.abyTe[0];
                scaabyResponseBuffer[byBufferIndex][AVERAGE_INDEX_H] = gvuwCalibrationSampleAvg.abyTe[1];
                scaabyResponseBuffer[byBufferIndex][AVERAGE_INDEX_L] = gvuwCalibrationSampleAvg.abyTe[0];
                    
                if (scbySlaveStatus == READ_ADC_PASSED)
                {
                    scTransmit(scaabyResponseBuffer[byBufferIndex], MAX_PACKET_SIZE);
                }
                else
                {
                    scTransmit(scaabyResponseBuffer[byBufferIndex], ADC_VALUE_INDEX_H);
                }
                
                scaabyResponseBuffer[byBufferIndex][STATUS_INDEX] = INVALID_STATUS;
                
                eSlaveCommand = INVALID_CMD;
                break;
                
            case REQ_POSITION_TIMER_CMD:
                scabyPositionTimerBuffer[MAX_INDEX_H] = gvuwCalibrationMaxThreshold.abyTe[1];
                scabyPositionTimerBuffer[MAX_INDEX_L] = gvuwCalibrationMaxThreshold.abyTe[0];
                scabyPositionTimerBuffer[MIN_INDEX_H] = gvuwCalibrationMinThreshold.abyTe[1];
                scabyPositionTimerBuffer[MIN_INDEX_L] = gvuwCalibrationMinThreshold.abyTe[0];
                scabyPositionTimerBuffer[AVER_INDEX_H] = gvuwCalibrationSampleAvg.abyTe[1];
                scabyPositionTimerBuffer[AVER_INDEX_L] = gvuwCalibrationSampleAvg.abyTe[0];
                scabyPositionTimerBuffer[TIME_INDEX_H] = gvuwTimer.abyTe[1];
                scabyPositionTimerBuffer[TIME_INDEX_L] = gvuwTimer.abyTe[0];
                scabyPositionTimerBuffer[TICKS_INDEX_H] = gvuwPositionTicks.abyTe[1];
                scabyPositionTimerBuffer[TICKS_INDEX_L] = gvuwPositionTicks.abyTe[0];
    
                scTransmit(scabyPositionTimerBuffer, POSITION_TIMER_BUFFER_SIZE);
                
                // reset position ticks
                gvuwPositionTicks.wOrd = 0;
                
                eSlaveCommand = INVALID_CMD;
                break;
                
            case INVALID_CMD:
            default:
                if (scfReceive((RECEIVED_MESSAGE *)&scstReceivedMessage))
                {
                    if ((scstReceivedMessage.Payload[SLAVE_ID_INDEX] == GLOBAL_ID) ||
                        (scstReceivedMessage.Payload[SLAVE_ID_INDEX] == UNIQUE_SLAVE))
                    {
                        eSlaveCommand = (COMMANDS_E)scstReceivedMessage.Payload[COMMAND_INDEX];
                    }
                    else
                    {
                        scstReceivedMessage = (RECEIVED_MESSAGE) {0};
                    }
                }
                break;
        }
    }

    return 0;
}


/*----------------------------------------------------------------------------

@Prototype: static void scdwDoMeasurePosition(WORD wSensorChannel)

@Description: Wait until the threshold is exceeded and return the timer value

@Parameters: WORD wSensorChannel - The sensor to read

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
06/21/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scDoMeasurePosition(WORD wSensorChannel)
{
    WORD wADCValue = 0;

    // Reset timer
    scTimerInterruptInit();
    while (TRUE)
    {
        wADCValue = scwADCRead(wSensorChannel, ADC_READ_DELAY);
        
        // As soon a the max or min threshold is exceeded,
        // store the CPU ticks then break
        if ((wADCValue > gvuwCalibrationMaxThreshold.wOrd) ||
            (wADCValue < gvuwCalibrationMinThreshold.wOrd) ||
            (MODE != MODE_JUMPER_ON)) // Break out of loop by user input
        {
            gvuwPositionTicks.wOrd = TMR1;
            break;
        }
    }
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
    BYTE byPacketIndex = 0;
    BYTE byBuffers = 0;
    WORD_BYTE_U uwAdcVal;

    // Reset timer
    scTimerInterruptInit();
    while (gvuwTimer.wOrd < TIME_TO_MEASURE_ADC_SLAVE)
    {
        uwAdcVal.wOrd = scwADCRead(wSensorChannel, ADC_READ_DELAY);

        // As soon a the max or min threshold is exceeded,
        // take ADC samples of size MAX_PACKET_SIZE*TOTAL_RESPONSE_BUFFERS
        if ((uwAdcVal.wOrd > gvuwCalibrationMaxThreshold.wOrd) ||
            (uwAdcVal.wOrd < gvuwCalibrationMinThreshold.wOrd))
        {
            fRetVal = TRUE;
            for (byBuffers = 0; byBuffers < TOTAL_RESPONSE_BUFFERS; byBuffers++)
            {
                for (byPacketIndex = ADC_VALUE_INDEX_H; byPacketIndex < MAX_PACKET_SIZE; byPacketIndex++)
                {
                    uwAdcVal.wOrd = scwADCRead(wSensorChannel, ADC_READ_DELAY);
                    scaabyResponseBuffer[byBuffers][byPacketIndex] = uwAdcVal.abyTe[1];
                    byPacketIndex++;
                    scaabyResponseBuffer[byBuffers][byPacketIndex] = uwAdcVal.abyTe[0];
                }
            }
        }
    }

    return fRetVal;
}


/*----------------------------------------------------------------------------

@Prototype: static void scDoCalibrateSensor(WORD wSensorChannel)

@Description: Calculate sample average and max threshold for sensor calibration

@Parameters: WORD wSensorChannel - The sensor to calibrate

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
05/17/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scDoCalibrateSensor(WORD wSensorChannel)
{
    WORD wPresentADCValue = 0;
    QWORD qwSumADCValues = 0;
    QWORD qwCount = 0;
    
    // Reset calibration values
    gvuwCalibrationMaxThreshold.wOrd = 0;
    gvuwCalibrationMinThreshold.wOrd = 0xFFFF;
    gvuwCalibrationSampleAvg.wOrd = 0;
    
    // reset timer
    scTimerInterruptInit();
    while (gvuwTimer.wOrd < ONE_SEC)
    {
        wPresentADCValue = scwADCRead(wSensorChannel, ADC_READ_DELAY);

        if (wPresentADCValue > gvuwCalibrationMaxThreshold.wOrd)
        {
            gvuwCalibrationMaxThreshold.wOrd = wPresentADCValue;
        }
        else if (wPresentADCValue < gvuwCalibrationMinThreshold.wOrd)
        {
            gvuwCalibrationMinThreshold.wOrd = wPresentADCValue;
        }
        qwSumADCValues += wPresentADCValue;
        qwCount++;
    }
    if (gvuwCalibrationMaxThreshold.wOrd < 1118)
    {
        gvuwCalibrationMaxThreshold.wOrd += 5;
    }
    if (gvuwCalibrationMinThreshold.wOrd > 5)
    {
        gvuwCalibrationMinThreshold.wOrd -= 5;
    }
    gvuwCalibrationSampleAvg.wOrd = (WORD)(qwSumADCValues/qwCount);
}


/*----------------------------------------------------------------------------

@Prototype: static void scTransmit(BYTE * pbyBuffer, BYTE byLength)

@Description: Tx Buffer

@Parameters: BYTE byLength - The length of bytes
             BYTE * pbyBuffer - Pointer to Buffer to Tx

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
06/21/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scTransmit(BYTE * pbyBuffer, BYTE byLength)
{
    BYTE byI;

    // Clear the transmit buffer
    MiApp_FlushTx();

    // Write new frame to transmit buffer
    for (byI = 0; byI < byLength; byI++)
    {
         MiApp_WriteData(pbyBuffer[byI]);
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

@Prototype: static WORD scwADCRead(WORD wADCChannel, WORD wDelay)

@Description: Read an ADC value from a given ADC channel

@Parameters: WORD wADCChannel - ADC Channel

@Returns: WORD: The ADC value


@Revision History:
DATE             NAME               REVISION COMMENT
04/12/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static WORD scwADCRead(WORD wADCChannel, WORD wDelay)
{
    WORD wADCVal = 0;

    //SPI1STAT = 0x0000;  // Disable SPI bus to read ADC

    AD1CHS = wADCChannel;           // set channel to measure
    _ADON = 1;        // turn ADC on for taking readings
    _SAMP = 1;       // start sampling
    Delay10us(wDelay);
    _SAMP = 0;       // stop sampling
    while (!_DONE);  // wait for ADC to complete
    wADCVal = ADC1BUF0;
    _ADON = 0;       // turn ADC off for before taking next reading

    //SPI1STAT = 0x8000;  // Enable SPI bus to talk radio

    return wADCVal;
}

