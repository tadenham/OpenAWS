#ifndef COMMON_H
    #define COMMON_H

    #include <stdio.h>
    #include <string.h>
    #include <math.h>
    #include <time.h>

    #include <sys/time.h>
    #include <sys/unistd.h>

    #include <driver/i2c.h>
    #include <driver/rtc_io.h>
    #include <driver/uart.h>
    #include <esp_adc/adc_cali.h>
    #include <esp_adc/adc_cali_scheme.h>
    #include <esp_adc/adc_oneshot.h>
    #include <esp_log.h>
    #include <esp_mac.h>
    #include <esp_sleep.h>
    #include <esp_timer.h>
    #include <freertos/event_groups.h>
    #include <nvs_flash.h>

    #include "esp_littlefs.h" // File system
    #include "settings.h"
    #include "ulp.h" // ULP
    #include "ulp_main.h" // ULP

    #include <assert.h>
    #include <stdbool.h>
    #include "sdkconfig.h"

    /* NimBLE stack APIs */
    #include "host/ble_hs.h"
    #include "host/ble_uuid.h"
    #include "host/util/util.h"
    #include "nimble/ble.h"
    #include "nimble/nimble_port.h"
    #include "nimble/nimble_port_freertos.h"

    #define TAG "OpenAWS"
    #define DEVICE_NAME "OpenAWS_Setup"

    //Define hardware types
    #define OPENAWS 0x01
    #define ANDGLOBAL 0x02
    #define LILYGO 0x03

    extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
    extern const uint8_t ulp_main_bin_end[]  asm("_binary_ulp_main_bin_end");

    extern EventGroupHandle_t event_group;
    extern char imei[16];

    extern float precip;

    struct station {
        uint8_t measure_interval_s;
        uint8_t observation_interval_min;
        uint8_t transmit_interval_min;
        uint8_t transmit_interval_min_default;
        uint8_t temp_sensor;
        bool rain_gauge;
        bool anemometer;
        uint8_t battery_type;
        bool enable_power_save;
        char time_zone[48];
    };

    struct modem {
        uint8_t network;
        uint8_t module;
        char *apn;
        uint8_t sleep_mode;
    };

    extern struct station station;

    extern struct modem modem;

    extern char operator[32];
    
    extern bool queued;

    extern char *AT_response;
#endif