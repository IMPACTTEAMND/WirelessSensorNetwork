#ifndef _MASTER_SLAVE_CONFIG_
#define _MASTER_SLAVE_CONFIG_

//TODO: EDIT THIS FOR NUMBER OF TOTAL SLAVES
#define NUMBER_OF_SLAVES 1

// Over 6 buffers causes timing issues
#define TOTAL_RESPONSE_BUFFERS 4

/* -- DEFINES and ENUMS -- */
#define CODE_VERSION 13

/* Microcontroller defines */
#define RF_TRANSCEIVER_CHANNEL 25
#define HUMIDITY_SENSOR_CHANNEL 1
#define EXTERNAL_SENSOR_CHANNEL 2
#define LIGHT_SENSOR_CHANNEL 3
#define ANALOG_CHANNEL EXTERNAL_SENSOR_CHANNEL

/* MISC */
#define BUTTON_ONE 1
#define BUTTON_TWO 2
#define UNDEFINED 0
#define HIGHEST_PRIORITY 7
#define MODE_JUMPER_ON 0
#define NO_DELAY 0
#define ADC_READ_DELAY 8  // In tens of microseconds

/* Timer 1 values */

//#define TEN_USEC 0xFFFA // 0XFFFF - 0x0005 = 5 pulses = 10 usec count at
                        // Fosc=8MHZ with Timer prescalar of 1:8;
//#define HUNDRED_USEC 0xFFCD  // 0XFFFF - 0x0032 = 50 pulses = 100 usec count at
                               // Fosc=8MHZ with Timer prescalar of 1:8;
//#define ONE_MSEC 0xFE0B  // 0XFFFF - 0x01F4 = 500 pulses = 1 msec count at
                         // Fosc=8MHZ with Timer prescalar of 1:8;
//#define HUNDRED_MSEC 0x3CAF    // 0xFFFF - 0xC350 = 50,000 pulses = 100 msec
                               // count at Fosc=8MHz with Timer prescalar of 1:8

#ifdef TEN_USEC
#define HUNDRED_US 10
#define ONE_MS 100
#define TEN_MS 1000
#define HUNDRED_MS 10000
#define ONE_SEC 100000
#define FIVE_SEC 500000
#define THIRTY_SEC 3000000
#define CONVERT_TO_SEC(TIME) (TIME/100000)
#define PULSES TEN_USEC
#endif
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
#ifdef ONE_MSEC
#define ONE_MS 1
#define TEN_MS 10
#define HUNDRED_MS 100
#define ONE_SEC 1000
#define FIVE_SEC 5000
#define THIRTY_SEC 30000
#define CONVERT_TO_SEC(TIME) (TIME/1000)
#define PULSES ONE_MSEC
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
    MAX_THRESHOLD_INDEX_H,
    MAX_THRESHOLD_INDEX_L,
    MIN_THRESHOLD_INDEX_H,
    MIN_THRESHOLD_INDEX_L,
    AVERAGE_INDEX_H,
    AVERAGE_INDEX_L,
    ADC_VALUE_INDEX_H,
    ADC_VALUE_INDEX_L,      
    MAX_PACKET_SIZE = 100
} RESPONSE_BUFFER_E;


typedef enum
{
    SLAVE_INDEX,
    MAX_INDEX_H,
    MAX_INDEX_L,
    MIN_INDEX_H,
    MIN_INDEX_L,
    AVER_INDEX_H,
    AVER_INDEX_L, 
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
    DO_CALIBRATION_CMD,       // 1
    DO_READ_ADC_CMD,          // 2
    DO_MEASURE_POSITION_CMD,  // 3
    REQ_BUFFER_CMD,           // 4
    REQ_POSITION_TIMER_CMD,   // 5
    INVALID_CMD
} COMMANDS_E;

typedef enum
{
    INVALID_STATUS,
    READ_ADC_FAILED,
    READ_ADC_PASSED
} SLAVE_STATUS_E;


#endif /* _MASTER_SLAVE_CONFIG_ */
