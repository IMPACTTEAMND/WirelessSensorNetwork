#ifndef _MASTER_SLAVE_CONFIG_
#define _MASTER_SLAVE_CONFIG_

//TODO: EDIT THIS FOR NUMBER OF TOTAL SLAVES
#define NUMBER_OF_SLAVES 1


/* -- DEFINES and ENUMS -- */
#define CODE_VERSION 13

/* Microcontroller defines */
#define RF_TRANSCEIVER_CHANNEL 25
#define HUMIDITY_SENSOR_CHANNEL 1
#define EXTERNAL_SENSOR_CHANNEL 2
#define LIGHT_SENSOR_CHANNEL 3
#define ANALOG_CHANNEL LIGHT_SENSOR_CHANNEL


// Over 6 buffers causes timing issues
#define TOTAL_RESPONSE_BUFFERS 8

/* MISC */
#define BUTTON_ONE 1
#define BUTTON_TWO 2
#define UNDEFINED 0
#define HIGHEST_PRIORITY 7
#define MODE_JUMPER_ON 0
#define NO_DELAY 0
#define ADC_READ_DELAY 8  // In tens of microseconds

/* Timer 1 values */
//#define HUNDRED_USEC 0xFFCD  // 0XFFFF - 0x0050= 50 pulses = 100 usec count at
                               // Fosc=8MHZ with Timer prescalar of 1:8;
#define HUNDRED_MSEC 0x3CAF    // 0xFFFF - 0xC350 = 50,000 pulses = 100 msec
                               // count at Fosc=8MHz with Timer prescalar of 1:8
#ifdef HUNDRED_USEC
#define HUNDRED_US 1
#define ONE_MS 10
#define TEN_MS 100
#define HUNDRED_MS 1000
#define ONE_SEC 10000
#define FIVE_SEC 50000
#define THIRTY_SEC 300000
#define CONVERT_TO_SEC(TIME) (TIME/10000)
#define PULSES HUNDRED_USEC
#endif
#ifdef HUNDRED_MSEC
#define HUNDRED_MS 1
#define FIVE_HUNDRED_MS 5
#define ONE_SEC 10
#define FIVE_SEC 50
#define THIRTY_SEC 300
#define ONE_MIN 600
#define FIVE_MIN 3000
#define CONVERT_TO_SEC(TIME) (TIME/10)
#define PULSES HUNDRED_MSEC
#endif

/* Time defines */
#define TIME_TO_MEASURE_ADC_SLAVE FIVE_SEC
#define TIME_TO_MEASURE_ADC_MASTER FIVE_SEC+ONE_SEC

enum
{
    SLAVE_0_ID,
    SLAVE_1_ID,
    SLAVE_2_ID,
    SLAVE_3_ID,
    SLAVE_4_ID,
    SLAVE_5_ID,
    SLAVE_6_ID,
    SLAVE_7_ID,
    SLAVE_8_ID,
    SLAVE_9_ID,
    SLAVE_10_ID,
    GLOBAL_ID = 0xFF
};


typedef enum
{
    SLAVE_ID_INDEX,
    COMMAND_INDEX,
    STATUS_INDEX = COMMAND_INDEX,
    BUFFER_INDEX,
    MAX_THRESHOLD_INDEX,
    MIN_THRESHOLD_INDEX,
    AVERAGE_INDEX,
    ADC_VALUE_INDEX,
    MAX_PACKET_SIZE = 100
} RESPONSE_BUFFER_E;


typedef enum
{
    SLAVE_INDEX,
    MAX_INDEX,
    MIN_INDEX,
    AVER_INDEX,    
    TICKS_BYTE_1_INDEX,
    TICKS_BYTE_2_INDEX,
    TICKS_BYTE_3_INDEX,
    TICKS_BYTE_4_INDEX,
    TICKS_BYTE_5_INDEX,
    TICKS_BYTE_6_INDEX,
    TICKS_BYTE_7_INDEX,
    TICKS_BYTE_8_INDEX,
    POSITION_TIMER_BUFFER_SIZE
} POSITION_BUFFER_E;


/* -- TYPEDEFS and STRUCTURES -- */
typedef enum
{
    RESERVED,
    DO_CALIBRATION_CMD,
    DO_READ_ADC_CMD,
    DO_MEASURE_POSITION_CMD,
    REQ_BUFFER_CMD,
    REQ_POSITION_TIMER_CMD,
    INVALID_CMD
} COMMANDS_E;

typedef enum
{
    INVALID_STATUS,
    READ_ADC_FAILED,
    READ_ADC_PASSED
} SLAVE_STATUS_E;


#endif /* _MASTER_SLAVE_CONFIG_ */
