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
volatile QWORD gqwTicks;
volatile QWORD gqwTempTicks;

/* -- STATIC VARIABLES -- */
static BYTE scbySlaveStatus;
static BYTE scaabyResponseBuffer[TOTAL_RESPONSE_BUFFERS][MAX_PACKET_SIZE];
static BYTE scabyPositionTimerBuffer[POSITION_TIMER_BUFFER_SIZE];

// We will not overflow since we will have only 10 bits in each of 10 ADCs. 1024*10=~11000
static WORD scwCalibrationMaxThreshold;
static WORD scwCalibrationMinThreshold;
static WORD scwCalibrationSampleAvg;


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
    gqwTicks++;

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
    scbySlaveStatus = INVALID_STATUS;
    gqwTicks = 0;
    gqwTempTicks = 0;
    
    scwCalibrationMaxThreshold = 0;
    scwCalibrationMinThreshold = 0xFFFF;
    scwCalibrationSampleAvg = 0;

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
                byBufferIndex = stReceivedMessage.Payload[BUFFER_INDEX];
                scaabyResponseBuffer[byBufferIndex][MAX_THRESHOLD_INDEX] = (BYTE)(scwCalibrationMaxThreshold >> 1);
                scaabyResponseBuffer[byBufferIndex][MIN_THRESHOLD_INDEX] = (BYTE)(scwCalibrationMinThreshold >> 1);
                scaabyResponseBuffer[byBufferIndex][AVERAGE_INDEX] = (BYTE)(scwCalibrationSampleAvg >> 1);
                    
                if (scbySlaveStatus == READ_ADC_PASSED)
                {
                    scTransmit(scaabyResponseBuffer[byBufferIndex], MAX_PACKET_SIZE);
                }
                else
                {
                    scTransmit(scaabyResponseBuffer[byBufferIndex], ADC_VALUE_INDEX);
                }
                
                scaabyResponseBuffer[byBufferIndex][STATUS_INDEX] = INVALID_STATUS;
                
                eSlaveCommand = INVALID_CMD;
                break;
                
            case REQ_POSITION_TIMER_CMD:
                scabyPositionTimerBuffer[SLAVE_INDEX] = UNIQUE_SLAVE;
                scabyPositionTimerBuffer[MAX_INDEX] = (BYTE)(scwCalibrationMaxThreshold >> 1);
                scabyPositionTimerBuffer[MIN_INDEX] = (BYTE)(scwCalibrationMinThreshold >> 1);
                scabyPositionTimerBuffer[AVER_INDEX] = (BYTE)(scwCalibrationSampleAvg >> 1);
                scTransmit(scabyPositionTimerBuffer, POSITION_TIMER_BUFFER_SIZE);
                
                eSlaveCommand = INVALID_CMD;
                break;
                
            case INVALID_CMD:
            default:
                if (scfReceive((RECEIVED_MESSAGE *)&stReceivedMessage))
                {
                    if ((stReceivedMessage.Payload[SLAVE_ID_INDEX] == GLOBAL_ID) ||
                        (stReceivedMessage.Payload[SLAVE_ID_INDEX] == UNIQUE_SLAVE))
                    {
                        eSlaveCommand = (COMMANDS_E)stReceivedMessage.Payload[COMMAND_INDEX];
                    }
                    else
                    {
                        stReceivedMessage = (RECEIVED_MESSAGE) {0};
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
    
    // reset timer
    gqwTicks = 0;
    while (1==1)
    {
        wADCValue = scwADCRead(wSensorChannel, ADC_READ_DELAY);
        
        // As soon a the max or min threshold is exceeded, break
        if ((wADCValue > scwCalibrationMaxThreshold) ||
            (wADCValue < scwCalibrationMinThreshold))
        {
            gqwTempTicks = gqwTicks;
            break;
        }
        // Break out of loop user input
        else if(MODE != MODE_JUMPER_ON)
        {
            break;
        }
    }
    
    scabyPositionTimerBuffer[TICKS_BYTE_1_INDEX] = (BYTE)((gqwTempTicks >> 56) & 0xFF);
    scabyPositionTimerBuffer[TICKS_BYTE_2_INDEX] = (BYTE)((gqwTempTicks >> 48) & 0xFF);
    scabyPositionTimerBuffer[TICKS_BYTE_3_INDEX] = (BYTE)((gqwTempTicks >> 40) & 0xFF);
    scabyPositionTimerBuffer[TICKS_BYTE_4_INDEX] = (BYTE)((gqwTempTicks >> 32) & 0xFF);
    scabyPositionTimerBuffer[TICKS_BYTE_5_INDEX] = (BYTE)((gqwTempTicks >> 24) & 0xFF);
    scabyPositionTimerBuffer[TICKS_BYTE_6_INDEX] = (BYTE)((gqwTempTicks >> 16) & 0xFF);
    scabyPositionTimerBuffer[TICKS_BYTE_7_INDEX] = (BYTE)((gqwTempTicks >> 8) & 0xFF);
    scabyPositionTimerBuffer[TICKS_BYTE_8_INDEX] = (BYTE)((gqwTempTicks >> 0) & 0xFF);
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
    BYTE byBuffers = 0;

    // Reset timer
    gqwTicks = 0;
    while (gqwTicks < TIME_TO_MEASURE_ADC_SLAVE)
    {
        wADCValue = scwADCRead(wSensorChannel, ADC_READ_DELAY);

        // As soon a the max or min threshold is exceeded,
        // take ADC samples of size MAX_PACKET_SIZE*TOTAL_RESPONSE_BUFFERS
        if ((wADCValue > scwCalibrationMaxThreshold) ||
            (wADCValue < scwCalibrationMinThreshold))
        {
            fRetVal = TRUE;
            while (wPacketIndex < MAX_PACKET_SIZE)
            {
                wADCValue = scwADCRead(wSensorChannel, ADC_READ_DELAY);
                scaabyResponseBuffer[byBuffers][wPacketIndex] = (BYTE)((wADCValue) >> 1);
                wPacketIndex++;
                if ((wPacketIndex == MAX_PACKET_SIZE) &&
                    (byBuffers < TOTAL_RESPONSE_BUFFERS))
                {
                    wPacketIndex = ADC_VALUE_INDEX;
                    byBuffers++;
                }
            }
            break;
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
    scwCalibrationMaxThreshold = 0;
    scwCalibrationMinThreshold = 0xFFFF;
    scwCalibrationSampleAvg = 0;
    
    // reset timer
    gqwTicks = 0;
    while (gqwTicks < ONE_SEC)
    {
        wPresentADCValue = scwADCRead(wSensorChannel, ADC_READ_DELAY);

        if (wPresentADCValue > scwCalibrationMaxThreshold)
        {
            scwCalibrationMaxThreshold = wPresentADCValue;
        }
        else if (wPresentADCValue < scwCalibrationMinThreshold)
        {
            scwCalibrationMinThreshold = wPresentADCValue;
        }
        qwSumADCValues += wPresentADCValue;
        qwCount++;
    }
    
    scwCalibrationSampleAvg = (WORD)(qwSumADCValues/qwCount);
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

