#ifndef STATION_H
    #define STATION_H

    #include "common.h"

    #define TEMP_BIT BIT1
    #define PRECIP_BIT BIT2
    #define WIND_BIT BIT3
    #define I2C_READY_BIT BIT7
    #define SETUP_DONE_BIT BIT8

    //I2C definitions
    #define I2C_MASTER_NUM 0
    #define I2C_MASTER_TIMEOUT_MS 1000 
    #define I2C_CLK_SPEED 100000 // 100 kHz

    //Define temp/humidity sensors
    #define SHT4x 0x01
    #define SHT3x 0x02
    #define DS18B20 0x03

    //Define pressure sensors
    #define BMP580 0x01
    #define BMP280 0x02

    //Define battery types
    #define LTO 0x01
    #define LFP 0x02
    #define LI_ION 0x03
    #define NA_ION 0x04

    #define ADC_MAX 4095
    #define S_TO_US 1000000

    #if HARDWARE == OPENAWS
        #define VOLTAGE_SCALE_FACTOR 2
    #elif HARDWARE == ANDGLOBAL
        #define VOLTAGE_SCALE_FACTOR 1
    #elif HARDWARE == LILYGO
        #define VOLTAGE_SCALE_FACTOR 2
    #endif

    void station_wake(void);
    void station_config(void);
    void station_sleep(void);
    void ULP_initialize(void);
    uint8_t *format_data(const struct tm current_time, const bool max_min);
    bool mount_file_system();
    void read_file(const char *file);
    void write_file(const uint8_t *data, const char *file);
    void unmount_file_system();
    int read_ADC(const uint8_t gpio);
#endif