#include "station.h"
#include "modem.h"

RTC_DATA_ATTR int8_t last_format_min = -1;
RTC_DATA_ATTR int8_t last_format_hour = -1;

EventGroupHandle_t event_group;

RTC_DATA_ATTR bool queued;

RTC_DATA_ATTR struct station station;
RTC_DATA_ATTR struct modem modem;

static struct tm get_time(void)
{
    time_t now;
    time(&now);
    setenv("TZ", station.time_zone, 1);
    tzset();
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    return timeinfo;
}

static void create_observation(const struct tm current_time)
{
    ESP_LOGI(TAG, "Creating observation...");
    const uint8_t *data = format_data(current_time, true); // Format observation
    if (current_time.tm_min % station.transmit_interval_min == 0) {
        if (!transmit_data(data)) {
            if (mount_file_system()) {
                write_file(data, "/sdcard/queue.txt");
                unmount_file_system();
                queued = true;
            }
        }
    } else {
        if (mount_file_system()) {
            write_file(data, "/sdcard/queue.txt");
            unmount_file_system();
            queued = true;
        }
    }
    if (current_time.tm_hour == 0) {
        precip = 0;
    }
}

void app_main(void)
{
    station_wake();

    if (station.temp_sensor) {
        xEventGroupWaitBits(event_group, TEMP_BIT, pdTRUE, pdTRUE, 1000 / portTICK_PERIOD_MS); // Wait for at least one temperature/humidity measurement
    }

    if (station.rain_gauge) {
        xEventGroupWaitBits(event_group, PRECIP_BIT, pdTRUE, pdTRUE, 500 / portTICK_PERIOD_MS); // Wait for at least one precipitation measurement
    }

    if (station.anemometer) {
        xEventGroupWaitBits(event_group, WIND_BIT, pdTRUE, pdTRUE, 500 / portTICK_PERIOD_MS); // Wait for at least one wind measurement
    }

    const struct tm current_time = get_time();

    ESP_LOGI(TAG, "The current date/time is: %d-%02d-%02d %02d:%02d:%02d", current_time.tm_year + 1900, current_time.tm_mon + 1, current_time.tm_mday, current_time.tm_hour, current_time.tm_min, current_time.tm_sec);

    if (station.transmit_interval_min < 60) {
        if ((current_time.tm_min % station.observation_interval_min == 0) && current_time.tm_min != last_format_min) {
            last_format_min = current_time.tm_min;
            create_observation(current_time);
        }
    } else {
        if (current_time.tm_min == 0 && current_time.tm_hour != last_format_hour) {
            last_format_hour = current_time.tm_hour;
            create_observation(current_time);
        }
    }
    station_sleep();
}
