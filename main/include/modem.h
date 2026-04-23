#ifndef MODEM_H
    #define MODEM_H

    #include "common.h"

    //Define network types
    #define WIFI 0x00
    #define CELLULAR 0x01
    #define SATELLITE 0x02

    //Define modules
    #define SIM7070 0x01
    #define A7670 0x02
    #define LBAD0XX1SC 0x03
    //#define ROCKBLOCK 0x04

    //Define modem sleep modes
    #define POWER_OFF 0x00
    #define MODEM_SLEEP 0x01
    #define PSM 0x02

    #define NITZ 0x01
    #define NTP 0x02

    #if HARDWARE == OPENAWS || HARDWARE == ANDGLOBAL
        #define MODEM_EN_PIN GPIO_NUM_25
        #define MODEM_PWRKEY_PIN GPIO_NUM_32
        #define MODEM_SLEEP_PIN GPIO_NUM_32
        #define MODEM_TXD_PIN GPIO_NUM_17
        #define MODEM_RXD_PIN GPIO_NUM_16
    #elif HARDWARE == LILYGO
        #define MODEM_EN_PIN GPIO_NUM_25
        #define MODEM_PWRKEY_PIN GPIO_NUM_4
        #define MODEM_FLIGHT_PIN GPIO_NUM_25
        #define MODEM_TXD_PIN GPIO_NUM_27
        #define MODEM_RXD_PIN GPIO_NUM_26
    #endif

    #define UART_NUM UART_NUM_1
    #define UART_BAUD 115200
    #define RX_BUF_SIZE 1024

    #define PROCEED_BIT BIT0 // AT command response received
    #define ERROR_BIT BIT6 

    //void modem_initialize(void);
    void modem_power_on(void);
    void modem_power_off(void);
    void uart_init(void);
    bool sendAT(const char *command, char *callback, const uint8_t timeout);
    void rx_task(void *arg);
    bool transmit_data(const uint8_t *data);
    bool connect_UDP(void);
    bool send_UDP(const uint8_t *data, const uint8_t len);
    bool get_NTP(void);
    
#endif

