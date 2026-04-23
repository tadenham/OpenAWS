#ifndef SETTINGS_H
    #define SETTINGS_H

    #define MEASURE_INTERVAL_S_DEFAULT 10
    #define OBSERVATION_INTERVAL_MIN_DEFAULT 5
    #define TRANSMIT_INTERVAL_MIN_DEFAULT 5

    #define TEMP_AVERAGING true
    #define MEASURE_PRESSURE true

    #define RAIN_GAUGE_PIN GPIO_NUM_39
    #define WIND_SPEED_PIN GPIO_NUM_36
    #define WIND_DIRECTION_PIN GPIO_NUM_34
    #define BATTERY_PIN GPIO_NUM_35

    #define SDA_PIN GPIO_NUM_21
    #define SCL_PIN GPIO_NUM_22

    #define CONFIG_PIN GPIO_NUM_4
    #define CONFIG_LED_PIN GPIO_NUM_2

    #define USE_INTERNAL_PULLUPS false

    #define HARDWARE OPENAWS
    //#define HARDWARE ANDGLOBAL
    //#define HARDWARE LILYGO

    #define DEFAULT_APN "iot.1nce.net"

    #define PRESSURE_SENSOR BMP580
    //#define PRESSURE_SENSOR BMP280

    //#define TIME_ZONE_DEFAULT "EST5EDT,M3.2.0,M11.1.0"
    #define TIME_ZONE_DEFAULT "Etc/UTC"

    #define BATTERY_TYPE_DEFAULT LTO
    #define ENABLE_POWER_SAVE true
#endif