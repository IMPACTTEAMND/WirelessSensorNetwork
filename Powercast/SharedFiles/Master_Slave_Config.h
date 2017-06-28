#ifndef _MASTER_SLAVE_CONFIG_
#define _MASTER_SLAVE_CONFIG_

/* -- DEFINES and ENUMS -- */
#define CODE_VERSION 13

/* Microcontroller defines */
#define RF_TRANSCEIVER_CHANNEL 25

/* MISC */
#define HIGHEST_PRIORITY 7

typedef union
{
    QWORD qwOrd;
    DWORD adwOrd[2];
    WORD awOrd[4];
    BYTE abyTe[8];
} QWORD_BYTES;

#endif /* _MASTER_SLAVE_CONFIG_ */
