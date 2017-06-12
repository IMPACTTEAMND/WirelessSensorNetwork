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
#define UNIQUE_SLAVE SLAVE_1_ID


/* -- GLOBAL VARIABLES -- */
volatile DWORD gdwADCTicks;
volatile DWORD gdwCalibrationTicks;

/* -- STATIC VARIABLES -- */
static BYTE scbySlaveStatus;
static BYTE scaabyResponseBuffer[MAX_PACKET_SIZE][TOTAL_RESPONSE_BUFFERS];

static WORD scawCalibrationRunningAvgValues[10];
// We will not overflow since we will have only 10 bits in each of 10 ADCs. 1024*10=~11000
static WORD scwCalibrationMaxThreshold;
static WORD scwCalibrationRunningAvg;


/* -- STATIC FUNCTION PROTOTYPES -- */
static void scMainInit(void);
static void scTimerInterruptInit (void);
static void scTransmitResponseBuffer(BYTE byLength, BYTE byBuffer);
static BOOL scfReceive(RECEIVED_MESSAGE *stReceiveMessageBuffer);
static WORD scwADCRead(WORD wChannel);
static void scCalibrateSensor(WORD wSensorChannel);
static BOOL scfDoReadADC(WORD wSensorChannel);


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
    gdwADCTicks++;
    gdwCalibrationTicks++;
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
    gdwADCTicks = UNDEFINED;
    gdwCalibrationTicks = UNDEFINED;
    scwCalibrationMaxThreshold = UNDEFINED;
    scwCalibrationRunningAvg = UNDEFINED;

    for (byI = 0; byI < (sizeof(scawCalibrationRunningAvgValues)/sizeof(scawCalibrationRunningAvgValues[0])); byI++)
    {
        scawCalibrationRunningAvgValues[byI] = UNDEFINED;
    }

    for (byI = 0; byI < MAX_PACKET_SIZE; byI++)
    {
        for (byJ = 0; byJ < TOTAL_RESPONSE_BUFFERS; byJ++)
        {
            scaabyResponseBuffer[byI][byJ] = UNDEFINED;
        }
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
    BYTE byBufferIndex;

    scMainInit();
    
    gdwCalibrationTicks = 0;
    while (gdwCalibrationTicks < ONE_SEC)
    {
        scCalibrateSensor(EXTERNAL_SENSOR_CHANNEL);
    }
    
    while(TRUE)
    {
        /* This function maintains the operation of the stack
           It should be called as often as possible. */
        P2PTasks();

        switch(eSlaveCommand)
        {
            case READ_ADC_CMD:
                if (scfDoReadADC(EXTERNAL_SENSOR_CHANNEL))
                {
                    scbySlaveStatus = READ_ADC_PASSED;
                }
                else
                {
                    scbySlaveStatus = READ_ADC_FAILED;
                }
                eSlaveCommand = INVALID_CMD;
                break;

            case REQ_BUFFER_CMD:
                byBufferIndex = stReceivedMessage.Payload[BUFFER_INDEX];
                scaabyResponseBuffer[SLAVE_ID_INDEX][byBufferIndex] = UNIQUE_SLAVE;
                scaabyResponseBuffer[STATUS_INDEX][byBufferIndex] = scbySlaveStatus;
                scaabyResponseBuffer[BUFFER_INDEX][byBufferIndex] = byBufferIndex;
                scaabyResponseBuffer[MAX_THRESHOLD_INDEX][byBufferIndex] = (BYTE)(scwCalibrationMaxThreshold >> 1);
                scaabyResponseBuffer[AVERAGE_INDEX][byBufferIndex] = (BYTE)(scwCalibrationRunningAvg >> 1);
                
                if (scbySlaveStatus == READ_ADC_PASSED)
                {
                    scTransmitResponseBuffer(MAX_PACKET_SIZE, byBufferIndex);
                }
                else
                {
                    scTransmitResponseBuffer(ADC_VALUE_INDEX, byBufferIndex);
                }

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

    // Reset ADC Timer
    gdwADCTicks = 0;

    while (gdwADCTicks < TIME_TO_MEASURE_ADC)
    {
        wADCValue = scwADCRead(wSensorChannel);

        // As soon a the max threshold is exceeded, take ADC samples of size MAX_PACKET_SIZE*TOTAL_RESPONSE_BUFFERS
        if (wADCValue > scwCalibrationMaxThreshold)
        {
            fRetVal = TRUE;
            while (wPacketIndex < MAX_PACKET_SIZE)
            {
                wADCValue = scwADCRead(wSensorChannel);
                scaabyResponseBuffer[wPacketIndex][byBuffers] = (BYTE)((wADCValue) >> 1);

                wPacketIndex++;
                if ((wPacketIndex == MAX_PACKET_SIZE) && (byBuffers < TOTAL_RESPONSE_BUFFERS))
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

@Prototype: static void scTransmitResponseBuffer(BYTE byLength, BYTE byBuffer)

@Description: Tx Buffer

@Parameters: BYTE byLength - The length of bytes
             BYTE byBuffer - Buffer to Tx

@Returns: void

@Revision History:
DATE             NAME               REVISION COMMENT
04/07/2017       Ali Haidous        Initial Revision

*----------------------------------------------------------------------------*/
static void scTransmitResponseBuffer(BYTE byLength, BYTE byBuffer)
{
    BYTE byI;

    // Clear the transmit buffer
    MiApp_FlushTx();

    // Write new frame to transmit buffer
    for (byI = 0; byI < byLength; byI++)
    {
         MiApp_WriteData(scaabyResponseBuffer[byI][byBuffer]);
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
    WORD wADCVal = 0;

    SPI1STAT = 0x0000;  // Disable SPI bus to read ADC

    AD1CHS = wADCChannel;           // set channel to measure
    _ADON = 1;        // turn ADC on for taking readings
    Delay10us(15); // Delay 150us
    _SAMP = 1;       // start sampling
    while (!_DONE);  // wait for ADC to complete
    wADCVal = ADC1BUF0;
    _ADON = 0;       // turn ADC off for before taking next reading

    SPI1STAT = 0x8000;  // Enable SPI bus to talk radio

    return wADCVal;
}

