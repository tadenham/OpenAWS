#include "station.h"
#include "modem.h"

static float temp = -9999;
static float humidity = -9999;
static float dew_point = -9999;
#if TEMP_AVERAGING // Maximum of 30 measurements for a 5-minute average (at 10 second measure interval)
    RTC_DATA_ATTR static float temp_i[30]; 
    RTC_DATA_ATTR static float humidity_i[30];
    RTC_DATA_ATTR static float dew_point_i[30];
#endif
RTC_DATA_ATTR static float max_temp = -9999;
RTC_DATA_ATTR static float min_temp = 9999;
RTC_DATA_ATTR static float max_humidity = -9999;
RTC_DATA_ATTR static float min_humidity = 9999;
RTC_DATA_ATTR static float max_dew_point = -9999;
RTC_DATA_ATTR static float min_dew_point = 9999;

RTC_DATA_ATTR float precip;
static float precip_rate;
RTC_DATA_ATTR static float max_precip_rate;
RTC_DATA_ATTR static uint32_t precip_count;
RTC_DATA_ATTR static float pulse_time_last_s; 

static float wind_speed = -9999;
static float wind_gust = -9999;
static float wind_direction = -9999;
RTC_DATA_ATTR static uint32_t wind_pulses_i[12];
RTC_DATA_ATTR static float wind_gust_i[12]; 
RTC_DATA_ATTR static float wind_direction_i[12];
RTC_DATA_ATTR static float max_wind_speed = -9999;
RTC_DATA_ATTR static float max_wind_gust = -9999;
RTC_DATA_ATTR static uint32_t wind_count;

RTC_DATA_ATTR static uint16_t ulp_wakeup_period_us;

static const esp_vfs_littlefs_conf_t conf = {
    .base_path = "/sdcard",
    .partition_label = "storage",
    .format_if_mount_failed = true,
    .dont_mount = false,
};

static esp_err_t i2c_master_init(void)
{
    const i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = USE_INTERNAL_PULLUPS ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .scl_pullup_en = USE_INTERNAL_PULLUPS ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_CLK_SPEED,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

int read_ADC(const uint8_t gpio)
{
    adc_unit_t unit;
    adc_channel_t channel;
    adc_oneshot_io_to_channel(gpio, &unit, &channel);
    adc_oneshot_unit_handle_t adc1_handle;
    const adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = unit,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    const adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    int adc_raw;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel, &config));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &adc_raw));
    //ESP_LOGI(TAG, "ADC Unit %d Channel[%d] Raw Data: %d", unit+1, channel, adc_raw);
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    return adc_raw;
}

static bool adc_calibration_init(const adc_unit_t unit, const adc_channel_t channel, const adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        if (!calibrated) {
            ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
            const adc_cali_curve_fitting_config_t cali_config = {
                .unit_id = unit,
                .chan = channel,
                .atten = atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
            if (ret == ESP_OK) {
                calibrated = true;
            }
        }
    #endif

    #if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        if (!calibrated) {
            const adc_cali_line_fitting_config_t cali_config = {
                .unit_id = unit,
                .atten = atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
            if (ret == ESP_OK) {
                calibrated = true;
            }
        }
    #endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        //ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

/* Initialize ultra low-power coprocessor for rain gauge and anemometer monitoring */
void ULP_initialize(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* GPIO used for pulse counting. */
    const int rtcio_num_rain = rtc_io_number_get(RAIN_GAUGE_PIN);
    assert(rtc_gpio_is_valid_gpio(RAIN_GAUGE_PIN) && "GPIO used for rain must be an RTC IO");

    const int rtcio_num_wind = rtc_io_number_get(WIND_SPEED_PIN);
    assert(rtc_gpio_is_valid_gpio(WIND_SPEED_PIN) && "GPIO used for wind must be an RTC IO");

    /* Initialize some variables used by ULP program.
     * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
     * These variables are declared in an auto generated header file,
     * 'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
     * These variables are located in RTC_SLOW_MEM and can be accessed both by the
     * ULP and the main CPUs.
     *
     * Note that the ULP reads only the lower 16 bits of these variables.
     */
    ulp_debounce_counter_rain = 1;
    ulp_debounce_counter_wind = 1;
    ulp_debounce_max_count_rain = 1;
    ulp_debounce_max_count_wind = 1;
    ulp_next_edge_rain = 1;
    ulp_next_edge_wind = 1;
    ulp_io_number_rain = rtcio_num_rain; /* map from GPIO# to RTC_IO# */
    ulp_io_number_wind = rtcio_num_wind; /* map from GPIO# to RTC_IO# */

    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
    if (station.rain_gauge) {
        ulp_wakeup_period_us = 10000;
        rtc_gpio_init(RAIN_GAUGE_PIN);
        rtc_gpio_set_direction(RAIN_GAUGE_PIN, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pulldown_dis(RAIN_GAUGE_PIN);
        rtc_gpio_pullup_dis(RAIN_GAUGE_PIN);
        rtc_gpio_hold_en(RAIN_GAUGE_PIN);
        precip_count = 3600 / station.measure_interval_s; // Number of pulses in an hour at current measurement interval
    }
    if (station.anemometer) {
        ulp_wakeup_period_us = 1000;
        rtc_gpio_init(WIND_SPEED_PIN);
        rtc_gpio_set_direction(WIND_SPEED_PIN, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pulldown_dis(WIND_SPEED_PIN);
        rtc_gpio_pullup_dis(WIND_SPEED_PIN);
        rtc_gpio_hold_en(WIND_SPEED_PIN);
        wind_count = 3600 / station.measure_interval_s;
    }

    /* Set ULP wake up period
     * Minimum pulse width has to be T * (ulp_debounce_counter + 1)
     */
    ulp_set_wakeup_period(0, ulp_wakeup_period_us);

    /* Start the program */
    err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

static void measure_temp(void *arg)
{
    #define SHT_ADDRESS 0x44
    i2c_cmd_handle_t cmd;
    while (1) {
        xEventGroupWaitBits(event_group, I2C_READY_BIT, pdTRUE, pdTRUE, 500 / portTICK_PERIOD_MS);
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SHT_ADDRESS << 1) | 0, true);
        if (station.temp_sensor == SHT4x) {
            i2c_master_write_byte(cmd, 0xFD, true);
        } else if (station.temp_sensor == SHT3x) {
            i2c_master_write_byte(cmd, 0x2400 >> 8, true);
            i2c_master_write_byte(cmd, 0x2400 & 0xFF, true);
        }
        i2c_master_stop(cmd);
        if (i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) == ESP_OK) {
            i2c_cmd_link_delete(cmd);
            vTaskDelay(30 / portTICK_PERIOD_MS); // Wait 30 ms for measurement
            uint8_t buffer[6];
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (SHT_ADDRESS << 1) | 1, true);
            i2c_master_read(cmd, buffer, 6, I2C_MASTER_LAST_NACK);
            i2c_master_stop(cmd);
            if (i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) == ESP_OK){
                i2c_cmd_link_delete(cmd);
                xEventGroupSetBits(event_group, I2C_READY_BIT);
                const uint16_t raw_temperature = (buffer[0] << 8) + buffer[1];
                const uint16_t raw_humidity = (buffer[3] << 8) + buffer[4];

                temp = raw_temperature * (175.0 / 65535) - 45; 
                if (station.temp_sensor == SHT4x) {
                    humidity = raw_humidity * (125.0 / 65535) - 6; 
                    if (humidity > 100) {
                        humidity = 100;
                    } else if (humidity < 0) {
                        humidity = 0;
                    }
                } else if (station.temp_sensor == SHT3x) {
                   humidity = raw_humidity * (100.0 / 65535);
                }
                dew_point = 1.8 * 243.04 * (log(humidity / 100) + ((17.625 * temp)/(243.04 + temp)))/(17.625 - log(humidity / 100)-((17.625 * temp)/(243.04 + temp))) + 32;
                temp = 1.8 * temp + 32;       
            } else {
                ESP_LOGE(TAG, "Failed to read temperature sensor");
                temp = -9999;
                humidity = -9999;
                dew_point = -9999;
            }
        } else {
            ESP_LOGE(TAG, "Failed to initialize temperature sensor");
            temp = -9999;
            humidity = -9999;
            dew_point = -9999;
        }

        #if TEMP_AVERAGING
            float temp_sum = 0;
            float humidity_sum = 0;
            float dew_point_sum = 0;
            uint8_t number_valid = 0;
            const uint8_t readings = 300 / station.measure_interval_s; // Number of measurements per 5-minute period

            for (uint8_t i = 0; i < readings; i++) {
                if (i < readings - 1) {
                    temp_i[i] = temp_i[i + 1];
                    humidity_i[i] = humidity_i[i + 1];
                    dew_point_i[i] = dew_point_i[i + 1];
                } else {
                    temp_i[i] = temp;
                    humidity_i[i] = humidity;
                    dew_point_i[i] = dew_point;
                }
                //ESP_LOGI(TAG, "Temp/Dew/Humidity [%d]: %.1f, %.1f, %.0f", i, temp_i[i], dew_point_i[i], humidity_i[i]);
                if (temp_i[i] > -9999) {
                    temp_sum = temp_sum + temp_i[i];
                    humidity_sum = humidity_sum + humidity_i[i];
                    dew_point_sum = dew_point_sum + dew_point_i[i];
                    number_valid++;
                }
            }

            if (number_valid >= 1) { 
                temp = temp_sum / number_valid;
                humidity = humidity_sum / number_valid;
                dew_point = dew_point_sum / number_valid;
                ESP_LOGI(TAG, "Mean temp/dew point/humidity (of %d measurements): %.1f °F, %.1f °F, %.0f %%", number_valid, temp, dew_point, humidity);
            }
        #endif

        if (temp > -9999) {
            if (temp > max_temp) {
                max_temp = temp;
            }
            if (temp < min_temp) {
                min_temp = temp;
            }

            if (dew_point > max_dew_point) {
                max_dew_point = dew_point;
            }
            if (dew_point < min_dew_point) {
                min_dew_point = dew_point;
            }

            if (humidity > max_humidity) {
                max_humidity = humidity;
            }
            if (humidity < min_humidity) {
                min_humidity = humidity;
            }
        }
        xEventGroupSetBits(event_group, TEMP_BIT);
        vTaskDelay(1000 * station.measure_interval_s / portTICK_PERIOD_MS);
    }
}

static void measure_precip(void *arg)
{
    while (1) {
        precip_count++;
        const uint32_t pulse_count_from_ulp = (ulp_edge_count_rain & UINT16_MAX) / 2;
        ulp_edge_count_rain = ulp_edge_count_rain % 2;
        if (pulse_count_from_ulp) {
            if (precip_count >= (60 / station.measure_interval_s)) { // If over 1 minute since last tip, use measurement count since last pulse to calculate precipitation rate
                pulse_time_last_s = fmin(precip_count * station.measure_interval_s, 3600); // Limit to one hour
            } else { // If less than 1 minute since last tip, use ULP timer to calculate precipitation rate
                pulse_time_last_s = (float)((ulp_pulse_last_rain & UINT16_MAX) * ulp_wakeup_period_us) / S_TO_US; // Time between last two tips
                const float pulse_time_min_sec = (float)((ulp_pulse_min_rain & UINT16_MAX) * ulp_wakeup_period_us) / S_TO_US; // Shortest time between tips since last measurement
                const float precip_rate_max_last = 36 / pulse_time_min_sec; // Highest precip rate since last measurement
                if (precip_rate_max_last > max_precip_rate) {
                    max_precip_rate = precip_rate_max_last;
                }
            }
            precip_count = 0; // Reset measurement counter
            precip = precip + (float)pulse_count_from_ulp / 100; // Increment precipitation total
        }
        if (precip_count < fmax(300 / station.measure_interval_s, 1.5 * pulse_time_last_s / station.measure_interval_s) && pulse_time_last_s) {
            precip_rate = 36 / pulse_time_last_s;
        } else {
            precip_rate = 0;
        }

        if (precip_rate > max_precip_rate) {
            max_precip_rate = precip_rate;
        }

        ulp_pulse_min_rain = 0;

        ESP_LOGI(TAG, "Precipitation accumulation/rate/max rate: %.2f in, %.2f in/hr, %.2f in/hr", precip, precip_rate, max_precip_rate);

        xEventGroupSetBits(event_group, PRECIP_BIT);

        vTaskDelay(1000 * station.measure_interval_s / portTICK_PERIOD_MS);   
    }
}

static void measure_wind(void *arg)
{
    const uint8_t readings = 120 / station.measure_interval_s; // Number of measurements per 2-minute period
    #define WIND_SCALE_FACTOR 2.5
    #define WIND_CUP_DIAMETER 4.5 // Diameter of wind cups in inches, measured at centerline of cups
    while (1) {
        wind_count++;
        //ESP_LOGI(TAG, "Wind edge count: %ld", ulp_edge_count_wind);
        const uint32_t wind_pulses = (ulp_edge_count_wind & UINT16_MAX) / 2;
        ulp_edge_count_wind = ulp_edge_count_wind % 2;

        //ESP_LOGI(TAG, "Wind pulses: %ld", wind_pulses);

        float gust;

        if (wind_pulses) {
            const float pulse_time_min_wind_s = (float)((ulp_pulse_min_wind & UINT16_MAX) * ulp_wakeup_period_us) / S_TO_US;
            if (wind_count < (60 / station.measure_interval_s) && ulp_pulse_min_wind) {
                gust = (60 / pulse_time_min_wind_s) * WIND_SCALE_FACTOR * 60 * 4.5 * 3.14159 / (12 * 5280);
            } else {
                gust = 0.1;
            }
            wind_count = 0;
        } else {
            gust = 0;
        }

        if (gust > max_wind_gust) {
            max_wind_gust = gust;
        }

        ulp_pulse_min_wind = 0;

        float wind_pulses_sum = 0;
        float wind_direction_sum = 0; 
        uint8_t number_valid = 0;

        float wind_direction_normalized[120 / station.measure_interval_s];

        for (uint8_t i = 0; i < readings; i++) {
            if (i < readings - 1) {
                wind_pulses_i[i] = wind_pulses_i[i + 1];
                wind_gust_i[i] = wind_gust_i[i + 1];
                wind_direction_i[i] = wind_direction_i[i + 1];
            } else {
                wind_pulses_i[i] = wind_pulses;
                wind_gust_i[i] = gust;
                wind_direction_i[i] = (float)(read_ADC(WIND_DIRECTION_PIN)) * 360 / 4096;
            }
            if (wind_gust_i[i] > wind_gust) {
                wind_gust = wind_gust_i[i];
            }
            if (i == 0) {
                wind_direction_normalized[i] = wind_direction_i[i];
            } else {
                if (fabsf(wind_direction_i[i] - wind_direction_normalized[i - 1]) > 180) {
                    if (wind_direction_i[i] < wind_direction_normalized[i - 1]) {
                        wind_direction_normalized[i] = wind_direction_i[i] + 360;
                    } else {
                        wind_direction_normalized[i] = wind_direction_i[i] - 360;
                    }
                } else {
                    wind_direction_normalized[i] = wind_direction_i[i];
                }
            }
            if (wind_direction_i[i] > -9999) {
                wind_pulses_sum = wind_pulses_sum + wind_pulses_i[i];
                wind_direction_sum = wind_direction_sum + wind_direction_normalized[i];
                number_valid++;
            }
        }

        if (number_valid >= 100 / station.measure_interval_s) { 
                wind_speed = (wind_pulses_sum / ((float)number_valid * station.measure_interval_s)) * WIND_SCALE_FACTOR * 60 * 60 * WIND_CUP_DIAMETER * 3.14159 / (12 * 5280);
                wind_direction = (int)(wind_direction_sum / number_valid) % 360;
                if (wind_speed > max_wind_speed) {
                    max_wind_speed = wind_speed;
                }
                if (wind_direction < 0) {
                    wind_direction = wind_direction + 360;
                }
        }

        ESP_LOGI(TAG, "Wind speed/wind gust/wind direction (of %d measurements): %.1f mph, %.1f mph, %.0f degrees", number_valid, wind_speed, wind_gust, wind_direction);

        xEventGroupSetBits(event_group, WIND_BIT);

        vTaskDelay(1000 * station.measure_interval_s / portTICK_PERIOD_MS);  
    }
}

#if MEASURE_PRESSURE
    static float measure_pressure(void)
    {
        float pressure = 0;
        #if PRESSURE_SENSOR == BMP580
            #define BMP_ADDRESS 0x46
            i2c_cmd_handle_t cmd;
            xEventGroupWaitBits(event_group, I2C_READY_BIT, pdTRUE, pdTRUE, 500 / portTICK_PERIOD_MS);
            cmd = i2c_cmd_link_create();
            ESP_ERROR_CHECK(i2c_master_start(cmd));
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (BMP_ADDRESS << 1) | 0, true));
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x36, true)); // Oversampling rate configuration
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x40, true)); // 1x oversampling, enable pressure measurement
            ESP_ERROR_CHECK(i2c_master_stop(cmd));
            if (i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to initialize BMP580");
                xEventGroupSetBits(event_group, I2C_READY_BIT);
                return pressure;
            };
            i2c_cmd_link_delete(cmd);

            cmd = i2c_cmd_link_create();
            ESP_ERROR_CHECK(i2c_master_start(cmd));
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (BMP_ADDRESS << 1) | 0, true));
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x37, true)); // Output data rate configuration 
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x2, true)); // Forced one-time measurement
            if (i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to configure BMP580");
                xEventGroupSetBits(event_group, I2C_READY_BIT);
                return pressure;
            }
            i2c_cmd_link_delete(cmd);

            uint8_t buffer[3];

            cmd = i2c_cmd_link_create();
            ESP_ERROR_CHECK(i2c_master_start(cmd));
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (BMP_ADDRESS << 1) | 0, true));
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x20, true));
            if ((i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS)) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to write to BMP580");
                xEventGroupSetBits(event_group, I2C_READY_BIT);
                return pressure;
            }
            i2c_cmd_link_delete(cmd);

            vTaskDelay(30 / portTICK_PERIOD_MS); // Wait 30 ms for measurement

            cmd = i2c_cmd_link_create();
            ESP_ERROR_CHECK(i2c_master_start(cmd));
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (BMP_ADDRESS << 1) | 1, true));
            ESP_ERROR_CHECK(i2c_master_read(cmd, buffer, 3, I2C_MASTER_LAST_NACK));
            ESP_ERROR_CHECK(i2c_master_stop(cmd));
            if ((i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS)) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read data from BMP580");
                xEventGroupSetBits(event_group, I2C_READY_BIT);
                return pressure;
            }
            i2c_cmd_link_delete(cmd);

            xEventGroupSetBits(event_group, I2C_READY_BIT);

            const uint32_t rawPressure = buffer[0] + (buffer[1] << 8) + (buffer[2] << 16);
            pressure = (rawPressure / 64) / 3386.39;
            ESP_LOGI(TAG, "Pressure: %.03f", pressure);
            return pressure;
        #elif PRESSURE_SENSOR == BMP280
            //
        #endif
    }
#endif

void station_wake(void)
{
    const esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause(); // Get wakeup cause
    event_group = xEventGroupCreate(); // Create event group
    i2c_master_init(); // Initialize I2C
    xEventGroupSetBits(event_group, I2C_READY_BIT); // I2C bus ready

    if (cause != ESP_SLEEP_WAKEUP_TIMER) { // Initial boot
        station_config();

        if (station.temp_sensor) {
            #if TEMP_AVERAGING
                for (uint8_t i = 0; i < (300 / station.measure_interval_s); i++) { // Initialize temp, humidity, and dew point arrays
                    temp_i[i] = -9999;
                    humidity_i[i] = -9999;
                    dew_point_i[i] = -9999;
                }
            #endif
        }

        if (station.anemometer) {
            for (uint8_t i = 0; i < (120 / station.measure_interval_s); i++) {
                wind_pulses_i[i] = -9999;
                wind_gust_i[i] = -9999;
                wind_direction_i[i] = -9999;
            }
        }
    }

    if (station.temp_sensor) {
        xTaskCreate(measure_temp, "measure_temp", 1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL); // Start temperature/humidity measuring task
    }

    if (station.rain_gauge) {
        xTaskCreate(measure_precip, "measure_precip", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL); // Start precipitation measuring task
    }

    if (station.anemometer) {
        xTaskCreate(measure_wind, "measure_wind", 1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL); // Start wind measuring task
    }
}

static float measure_battery(void)
{
    adc_unit_t unit;
    adc_channel_t channel;
    adc_oneshot_io_to_channel(BATTERY_PIN, &unit, &channel);
    adc_oneshot_unit_handle_t adc1_handle;
    const adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = unit,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    const adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    int adc_raw[10];
    int voltage[10];
    int voltage_sum = 0;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel, &config));
    adc_cali_handle_t adc1_cali_handle = NULL;
    adc_calibration_init(unit, channel, ADC_ATTEN_DB_12, &adc1_cali_handle);
    for (int i = 0; i <= 9; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &adc_raw[i]));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", unit+1, channel, adc_raw[i]);
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[i], &voltage[i]));
        //ESP_LOGI(TAG, "ADC Unit %d Channel[%d] Cali Voltage: %d mV", unit+1, channel, voltage[i]);
        voltage_sum = voltage_sum + voltage[i];
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    const float vbat = VOLTAGE_SCALE_FACTOR * ((float)voltage_sum / 10000);
    #if ENABLE_POWER_SAVE
        float vbat_critical; // 25% charged
        float vbat_low; // 50% charged
        float vbat_med; // 75% charged
        switch (station.battery_type) {
            case LTO:
                vbat_critical = 2.20;
                vbat_low = 2.40;
                vbat_med = 2.55;
                break;
            case LI_ION:
                vbat_critical = 3.00;
                vbat_low = 3.70;
                vbat_med = 3.95;
                break;
            default:
                vbat_critical = 2.20;
                vbat_low = 2.40;
                vbat_med = 2.55;
        }
        if (vbat > vbat_med || vbat < 1.0) {
            station.transmit_interval_min = station.transmit_interval_min_default;
        } else if (vbat > vbat_low) {
            station.transmit_interval_min = fmax(station.transmit_interval_min_default, 15);
        } else if (vbat > vbat_critical) {
            station.transmit_interval_min = fmax(station.transmit_interval_min_default, 30);
        } else {
            station.transmit_interval_min = fmax(station.transmit_interval_min_default, 60);
        }
    #endif
    ESP_LOGI(TAG, "Battery voltage: %.3f V", vbat);
    return vbat;
}

/**
 * @brief Format message for transmission
 * 
 * @param current_time Timestamp for data
 * @param max_min Whether to transmit max/min values
 */
uint8_t *format_data(const struct tm current_time, const bool max_min)
{
    ESP_LOGI(TAG, "Formatting observation...");
    uint8_t *data = malloc(35);

    bool request_response = true;

    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);

    data[0] = (request_response << 7) | (max_min << 6) | (mac[0] >> 2);
    data[1] = ((mac[0] << 6) | (mac[1] >> 2)) & 0xFF;
    data[2] = ((mac[1] << 6) | (mac[2] >> 2)) & 0xFF;
    data[3] = ((mac[2] << 6) | (mac[3] >> 2)) & 0xFF;
    data[4] = ((mac[3] << 6) | (mac[4] >> 2)) & 0xFF;
    data[5] = ((mac[4] << 6) | (mac[5] >> 2)) & 0xFF;

    data[6] = ((mac[5] << 6) | ((current_time.tm_year % 100) >> 1)) & 0xFF;
    data[7] = (((current_time.tm_year % 100) << 7) | ((current_time.tm_mon + 1) << 3) | (current_time.tm_mday >> 2)) & 0xFF;
    data[8] = ((current_time.tm_mday << 6) | (current_time.tm_hour << 1) | ((current_time.tm_min / 5) >> 3)) & 0xFF;

    const float battery_voltage = measure_battery();

    data[9] = (((current_time.tm_min / 5) << 5) | ((uint16_t)(lrintf(battery_voltage * 100)) >> 4)) & 0xFF;

    const float pressure = measure_pressure();
    data[10] = (((uint16_t)(lrintf(battery_voltage * 100)) << 4) | ((uint16_t)(lrintf(pressure * 100)) >> 8)) & 0xFF;
    data[11] = ((uint16_t)(lrintf(pressure * 100)) & 0xFF);
        
    if (temp == -9999) {
        temp = 200;
        max_temp = 200;
        min_temp = 200;
    }

    const uint8_t sign_T = temp >= 0 ? 0 : 1;
    const uint8_t sign_Tx = max_temp >= 0 ? 0 : 1;
    const uint8_t sign_Tn = min_temp >= 0 ? 0 : 1;

    data[12] = (sign_T << 7) | ((uint16_t)(labs(lrintf(temp * 10))) >> 4);
    data[13] = (((uint16_t)(labs(lrintf(temp * 10))) << 4) | (sign_Tx << 3) | ((uint16_t)(labs(lrintf(max_temp * 10))) >> 8)) & 0xFF;
    data[14] = ((uint16_t)(labs(lrintf(max_temp * 10))) & 0xFF);
    data[15] = (sign_Tn << 7) | ((uint16_t)(labs(lrintf(min_temp * 10))) >> 4);

    if (dew_point == -9999) {
        dew_point = 200;
        max_dew_point = 200;
        min_dew_point = 200;
    }

    const uint8_t sign_D = dew_point >= 0 ? 0 : 1;
    const uint8_t sign_Dx = max_dew_point >= 0 ? 0 : 1;
    const uint8_t sign_Dn = min_dew_point >= 0 ? 0 : 1;

    data[16] = (((uint16_t)(labs(lrintf(min_temp * 10))) << 4) | (sign_D << 3) | ((uint16_t)(labs(lrintf(dew_point * 10))) >> 8)) & 0xFF;
    data[17] = (uint16_t)(labs(lrintf(dew_point * 10))) & 0xFF;
    data[18] = (sign_Dx << 7) | ((uint16_t)(labs(lrintf(max_dew_point * 10))) >> 4);
    data[19] = (((uint16_t)(labs(lrintf(max_dew_point * 10))) << 4) | (sign_Dn << 3) | ((uint16_t)(labs(lrintf(min_dew_point * 10))) >> 8)) & 0xFF;
    data[20] = ((uint16_t)(labs(lrintf(min_dew_point * 10))) & 0xFF);

    if (humidity == -9999) {
        humidity = 101;
        max_humidity = 101;
        min_humidity = 101;
    }

    data[21] = ((uint8_t)(lrintf(humidity)) << 1) | ((uint8_t)(lrintf(max_humidity)) >> 6);
    data[22] = (((uint8_t)(lrintf(max_humidity)) << 2) | ((uint8_t)(lrintf(min_humidity)) >> 5)) & 0xFF;

    if (!station.rain_gauge) {
        precip = 80;
        precip_rate = 10;
        max_precip_rate = 10;
    }

    data[23] = (((uint8_t)(lrintf(min_humidity)) << 3) | ((uint16_t)(lrintf(fmin(precip, 80) * 100)) >> 10)) & 0xFF;
    data[24] = ((uint16_t)(lrintf(fmin(precip, 80) * 100)) >> 2) & 0xFF;
    data[25] = (((uint16_t)(lrintf(fmin(precip, 80) * 100)) << 6) | ((uint16_t)(lrintf(fmin(precip_rate, 10) * 100)) >> 4)) & 0xFF;
    data[26] = (((uint16_t)(lrintf(fmin(precip_rate, 10) * 100)) << 4) | ((uint16_t)(lrintf(fmin(max_precip_rate, 10) * 100)) >> 6)) & 0xFF;

    if (wind_speed == -9999) {
        wind_speed = 400;
        max_wind_speed = 400;
        wind_gust = 400;
        max_wind_gust = 400;
        wind_direction = 360;
    }

    data[27] = (((uint16_t)(lrintf(fmin(max_precip_rate, 10) * 100)) << 2) | ((uint16_t)(lrintf(fmin(wind_speed, 400) * 10)) >> 10)) & 0xFF;
    data[28] = ((uint16_t)(lrintf(fmin(wind_speed, 400) * 10)) >> 2) & 0xFF;
    data[29] = (((uint16_t)(lrintf(fmin(wind_speed, 400) * 10)) << 6) | ((uint16_t)(lrintf(fmin(wind_gust, 400) * 10)) >> 6)) & 0xFF;
    data[30] = (((uint16_t)(lrintf(fmin(wind_gust, 400) * 10)) << 2) | ((uint16_t)(lrintf(wind_direction)) >> 7)) & 0xFF;
    data[31] = (((uint16_t)(lrintf(wind_direction)) << 1) | ((uint16_t)(lrintf(fmin(max_wind_speed, 400) * 10)) >> 11)) & 0xFF;
    data[32] = ((uint16_t)(lrintf(fmin(max_wind_speed, 400) * 10)) >> 3) & 0xFF;
    data[33] = (((uint16_t)(lrintf(fmin(max_wind_speed, 400) * 10)) << 5) | ((uint16_t)(lrintf(fmin(max_wind_gust, 400) * 10)) >> 7)) & 0xFF;
    data[34] = ((uint16_t)(lrintf(fmin(max_wind_gust, 400) * 10)) << 1) & 0xFF;

    max_temp = -9999;
    min_temp = 9999;
    max_dew_point = -9999;
    min_dew_point = 9999;
    max_humidity = -9999;
    min_humidity = 9999;
    max_precip_rate = 0;
    max_wind_speed = -9999;
    max_wind_gust = -9999;

    return data;
}

bool mount_file_system()
{
    ESP_LOGI(TAG, "Initializing LittleFS");

    // Use settings defined above to initialize and mount LittleFS filesystem.
    // Note: esp_vfs_littlefs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
            return false;
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
            return false;
        } else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
            return false;
        }
        return false;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
        esp_littlefs_format(conf.partition_label);
        return false;
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
        return true;
    }
}

void read_file(const char *file)
{
    ESP_LOGI(TAG, "Reading file %s", file);
    FILE *f = fopen(file, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }

    // Read a line from file
    //char line[256];

    uint8_t line[35];

    queued = false;

    while (fread(line, 1, 35, f) == 35) {
    //while (fgets(line, sizeof(line), f)) {
        /*char *pos = strchr(line, '\n');
        if (pos) {
            *pos = '\0';
        }*/
        //ESP_LOGI(TAG, "Read from file: '%s'", line);
        if (esp_timer_get_time() < (station.observation_interval_min * 30 * S_TO_US)) { // Allow up to half the measuring period to transmit stored data
            if (queued) {
                write_file(line, "/sdcard/failed.txt");
            } else {
                if (!send_UDP((uint8_t *)line, 35)) {
                    queued = true;
                    write_file(line, "/sdcard/failed.txt");
                }
            }
        } else {
            ESP_LOGE(TAG, "Time limit exceeded");
            queued = true;
            write_file(line, "/sdcard/failed.txt"); 
        }
    }
    fclose(f);
}

void write_file(const uint8_t *data, const char *file)
{
    ESP_LOGI(TAG, "Opening file %s", file);
    FILE *f = fopen(file, "ab");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for appending");
        return;
    }
    //fprintf(f, "%s\n", data);
    fwrite(data, 1, 35, f);
    fclose(f);
    ESP_LOGI(TAG, "File written");
}

void unmount_file_system()
{
    // All done, unmount partition and disable LittleFS
    esp_vfs_littlefs_unregister(conf.partition_label);
    ESP_LOGI(TAG, "LittleFS unmounted");
}

void station_sleep()
{
    #define BOOT_TIME_US 140000 // Time required to boot in microseconds. 250000 with image validation enabled
    ESP_LOGI(TAG, "Entering deep sleep...");
    const uint64_t sleep_time_us = (station.measure_interval_s * S_TO_US) - (esp_timer_get_time() % (station.measure_interval_s * S_TO_US)) - BOOT_TIME_US; // Measurement interval minus time elapsed since boot
    esp_sleep_enable_timer_wakeup(sleep_time_us); // Enable deep sleep timer
    esp_deep_sleep_start(); // Enter deep sleep
}