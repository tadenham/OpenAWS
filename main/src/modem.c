#include "station.h"
#include "modem.h"

char imei[16];
char operator[32];

char *AT_response = "";

#if MODEM_TYPE == SATELLITE
    static char set_time_cmd[64];
#endif

/**
* @brief Initializes UART for communication with module
*/
void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 1024, 0, NULL, ESP_INTR_FLAG_IRAM);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, MODEM_TXD_PIN, MODEM_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/**
* @brief Set time from modem response
* @param s Pointer to AT command response to extract time from
*/
static void set_time(const char *s)
{
    time_t timeSinceEpoch = 0;

    //ESP_LOGI(TAG, "Time from modem: %s", s);
    //CCLK: "+CCLK: "23/06/10,14:09:19-16"
    char year[3];
    strncpy(year, s + 7, 2); 
    const int yearint = atoi(year) + 100; // Years since 1900

    char month[3];
    strncpy(month, s + 10, 2);  
    const int monthint = atoi(month) - 1; // Jan = 0

    char day[3];
    strncpy(day, s + 13, 3);  
    const int dayint = atoi(day);

    char hour[3];
    strncpy(hour, s + 16, 2);  
    const int hourint = atoi(hour);

    char min[3];
    strncpy(min, s + 19, 2);  
    const int minint = atoi(min);

    char sec[3];
    strncpy(sec, s + 22, 2);  
    const int secint = atoi(sec);

    struct tm t = {0};
    t.tm_year = yearint;
    t.tm_mon = monthint;
    t.tm_mday = dayint;
    t.tm_hour = hourint;
    t.tm_min = minint;
    t.tm_sec = secint;
    unsetenv("TZ");
    timeSinceEpoch = mktime(&t);
    struct timeval tv;
    tv.tv_sec = timeSinceEpoch;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);
    #if MODEM_TYPE == SATELLITE
        //xEventGroupSetBits(event_group, PROCEED_BIT);   
    #endif
}

/**
* @brief Set time from unix timestamp from server
* @param unix Pointer to 6-byte Unix timestamp
*/
static void set_time_from_unix(uint8_t *unix)
{
    const time_t timeSinceEpoch = (unix[0] & UINT64_MAX) << 40 | (unix[1] & UINT64_MAX) << 32 | (unix[2] & UINT64_MAX) << 24 | (unix[3] & UINT64_MAX) << 16 | (unix[4] & UINT64_MAX) << 8  | (unix[5] & UINT64_MAX);
    ESP_LOGI(TAG, "Unix time: %lld", timeSinceEpoch);
    struct timeval tv;
    tv.tv_sec = timeSinceEpoch;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);
}

#if MODEM_TYPE == SATELLITE
    static void set_modem_clock(const char *s)
    {
        //%IGNSSINFO: 2,"23:48:57","18/02/2026","27.110288","-80.279875","14.5",1771458537000,,,"B",4
        char year[3];
        strncpy(year, s + 34, 2);
        year[2] = 0;

        char month[3];
        strncpy(month, s + 29, 2);
        month[2] = 0;

        char day[3];
        strncpy(day, s + 26, 2);
        day[2] = 0;
        
        char hour[3];
        strncpy(hour, s + 15, 2);
        hour[2] = 0;

        char min[3];
        strncpy(min, s + 18, 2);  
        min[2] = 0;

        char sec[3];
        strncpy(sec, s + 21, 2);
        sec[2] = 0;

        strcpy(set_time_cmd, "AT+CCLK=\"");
        strcat(set_time_cmd, year);
        strcat(set_time_cmd, "/");
        strcat(set_time_cmd, month);
        strcat(set_time_cmd, "/");
        strcat(set_time_cmd, day);
        strcat(set_time_cmd, ",");
        strcat(set_time_cmd, hour);
        strcat(set_time_cmd, ":");
        strcat(set_time_cmd, min);
        strcat(set_time_cmd, ":");
        strcat(set_time_cmd, sec);
        strcat(set_time_cmd, "+00\"\r");
    }
#endif

static void set_operator(const char *s)
{
    strcpy(operator, s + 1);
    operator[strlen(operator) - 1] = '\0'; // Remove last character
    ESP_LOGI(TAG, "Operator: %s", operator);
}

/**
* @brief Receive AT command responses from modem
*/
void rx_task(void *arg)
{
    char *data = (char*)malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM, data, RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            char *token = strtok(data, "\n");
            while (token != NULL) {
                ESP_LOGI("Modem", "%s", token);
                char *s;
                s = strstr(token, AT_response);
                if (s != NULL) {
                    if (strcmp(AT_response, "CCLK:") == 0) {
                        set_time(s);
                    }
                    if (strcmp(AT_response, "COPS:") == 0) {
                        char *delim = strtok(s, ",");
                        uint8_t i = 0;
                        while (delim != NULL) {
                            if (i == 2) {
                                set_operator(delim);
                                break;
                            }
                            delim = strtok(NULL, ",");
                            i++;
                        }
                    }
                    #if MODEM_TYPE == SATELLITE
                        if (strcmp(AT_response, "%IGNSSINFO:") == 0) {
                            set_modem_clock(s);
                        }
                    #endif
                    if (strcmp(AT_response, "AT+CGSN") != 0 && strcmp(AT_response, "AT+CGMM") != 0 && strcmp(AT_response, "+CAURC: \"recv\"") != 0) {
                        xEventGroupSetBits(event_group, PROCEED_BIT);
                    }
                }
                if (strcmp(AT_response, "AT+CGSN") == 0 && strlen(token) == 16) {
                    strcpy(imei, token);
                    xEventGroupSetBits(event_group, PROCEED_BIT);
                }
                if (strcmp(AT_response, "AT+CGMM") == 0 && (strstr(token, "SIM7070") != NULL || strstr(token, "A7670") != NULL || strstr(token, "LBAD0XX1SC") != NULL)) {
                    if (strstr(token, "SIM7070") != NULL) {
                        modem.module = SIM7070;
                    } else if (strstr(token, "A7670") != NULL) {
                        modem.module = A7670;
                    } else if (strstr(token, "LBAD0XX1SC") != NULL) {
                        modem.module = LBAD0XX1SC;
                    }
                    xEventGroupSetBits(event_group, PROCEED_BIT);
                }
                /*s = strstr(token, error_response);
                if (s != NULL){
                    xEventGroupSetBits(event_group, ERROR_BIT);
                }*/
                token = strtok(NULL, "\n");
            }
            if (strcmp(AT_response, "+CAURC: \"recv\"") == 0 && rxBytes == 30) {
                uint8_t time_data[6];
                time_data[0] = data[22];
                time_data[1] = data[23];
                time_data[2] = data[24];
                time_data[3] = data[25];
                time_data[4] = data[26];
                time_data[5] = data[27];
                set_time_from_unix(time_data);
                xEventGroupSetBits(event_group, PROCEED_BIT);
            }
        }
    }
    free(data);
}

void modem_power_on(void)
{
    ESP_LOGI(TAG, "Starting modem...");
    #if HARDWARE == OPENAWS
        gpio_pulldown_dis(MODEM_EN_PIN);
        //gpio_set_direction(MODEM_EN_PIN, GPIO_MODE_OUTPUT);
        gpio_pullup_en(MODEM_EN_PIN);
        //gpio_set_level(MODEM_EN_PIN, 1);
        gpio_pullup_en(MODEM_PWRKEY_PIN);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_pullup_dis(MODEM_PWRKEY_PIN);
        //gpio_hold_en(MODEM_EN_PIN);
        //gpio_deep_sleep_hold_en();
    #elif HARDWARE == ANDGLOBAL
        gpio_pulldown_dis(MODEM_EN_PIN);
        gpio_pullup_en(MODEM_EN_PIN);
        gpio_pulldown_en(MODEM_PWRKEY_PIN);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_pulldown_dis(MODEM_PWRKEY_PIN);
    #elif HARDWARE == LILYGO
        //gpio_pullup_en(MODEM_PWRKEY_PIN);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        //gpio_pullup_dis(MODEM_PWRKEY_PIN);
        gpio_set_direction(MODEM_EN_PIN, GPIO_MODE_OUTPUT);
        //gpio_set_direction(MODEM_PWRKEY_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(MODEM_EN_PIN, 1);
        gpio_pullup_en(MODEM_PWRKEY_PIN);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_pullup_dis(MODEM_PWRKEY_PIN);
        //gpio_set_level(MODEM_PWRKEY_PIN, 1);
        //vTaskDelay(500 / portTICK_PERIOD_MS);
        //gpio_set_level(MODEM_PWRKEY_PIN, 0);
        gpio_hold_en(MODEM_EN_PIN);
        gpio_deep_sleep_hold_en();
    #endif
}

/**
* @brief Send AT command to modem and waits for specified response
* @param command Command to send. Must be terminated by \r.
* @param callback Modem response to wait for
* @param timeout Seconds to wait for response before giving up. Setting timeout to 0 allows the function to wait indefinitely.
*/
bool sendAT(const char *command, char *callback, const uint8_t timeout)
{
    const int len = strlen(command);
    AT_response = callback;
    uart_write_bytes(UART_NUM, command, len);
    EventBits_t uxBits;
    if (timeout == 0) {
        uxBits = xEventGroupWaitBits(event_group, PROCEED_BIT | ERROR_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    } else {
        uxBits = xEventGroupWaitBits(event_group, PROCEED_BIT | ERROR_BIT, pdTRUE, pdFALSE, timeout * 1000 / portTICK_PERIOD_MS);
    }
    if ((uxBits & PROCEED_BIT) != 0) { // Received correct response
        //ESP_LOGI(TAG, "AT command successful");
        return true;
    } else if ((uxBits & ERROR_BIT) != 0) { // Received error
        //ESP_LOGE(TAG, "AT command fail: incorrect response");
        return false;
    } else { // Timeout
        ESP_LOGE(TAG, "AT command (%s) timeout", command);
        return false;
    };
}

bool send_bytes(const uint8_t *data, const uint8_t len, char *callback, const uint8_t timeout)
{
    AT_response = callback;
    uart_write_bytes(UART_NUM, data, len);
    EventBits_t uxBits;
    if (timeout == 0) {
        uxBits = xEventGroupWaitBits(event_group, PROCEED_BIT | ERROR_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    } else {
        uxBits = xEventGroupWaitBits(event_group, PROCEED_BIT | ERROR_BIT, pdTRUE, pdFALSE, timeout * 1000 / portTICK_PERIOD_MS);
    }
    if ((uxBits & PROCEED_BIT) != 0) { // Received correct response
        //ESP_LOGI(TAG, "AT command successful");
        return true;
    } else if ((uxBits & ERROR_BIT) != 0) { // Received error
        //ESP_LOGE(TAG, "AT command fail: incorrect response");
        return false;
    } else { // Timeout
        //ESP_LOGE(TAG, "AT command (%s) timeout", command);
        return false;
    };
}

void modem_power_off(void)
{
    ESP_LOGI(TAG, "Powering off modem...");
    if (modem.module == SIM7070) {
        sendAT("AT+CPOWD=1\r", "NORMAL POWER DOWN", 1);
    } else if (modem.module == A7670) {
        sendAT("AT+CPOF\r", "OK", 1);
    } else if (modem.module == LBAD0XX1SC) {
        sendAT("AT+CFUN=0\r", "OK", 1);
    }
    #if HARDWARE == OPENAWS || HARDWARE == ANDGLOBAL
        gpio_pullup_dis(MODEM_EN_PIN);
        gpio_pulldown_en(MODEM_EN_PIN);
    #endif
}

bool get_NTP(void)
{
    if (modem.module == SIM7070) {
        sendAT("AT+CNTP=\"time.nist.gov\",0\r", "OK", 5);
        if (sendAT("AT+CNTP\r", "+CNTP: 1", 5)) {
            return true;
        } else {
            return false;
        }
    } else if (modem.module == A7670) {
        sendAT("AT+CNTP=\"pool.ntp.org\",0\r", "OK", 5);
        if (sendAT("AT+CNTP\r", "+CNTP: 0", 5)) {
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

bool connect_UDP(void)
{
    const uint8_t network_timeout_s = 20; // Use 20 seconds for normal signal, 60 seconds for weak signal
    const uint8_t connect_timeout_s = 10;
    if (modem.module == SIM7070) {
        if (sendAT("AT+CNACT=0,1\r", "+APP PDP: 0,ACTIVE", network_timeout_s)) {
            if (sendAT("AT+CAOPEN=1,0,\"UDP\",\"142.11.236.169\",5255,1\r", "+CAOPEN: 1,0", connect_timeout_s)) {
                ESP_LOGI(TAG, "UDP connnection successful");
                return true;
            } else {
                ESP_LOGE(TAG, "UDP connnection failed");
                return false;
            }
        } else {
            ESP_LOGE(TAG, "UDP connnection failed");
            return false;
        }
    } else if (modem.module == A7670) {
        sendAT("AT+NETOPEN\r", "OK", network_timeout_s);
        if (sendAT("AT+CIPOPEN=1,\"UDP\",,,5000\r", "+CIPOPEN: 1,0", connect_timeout_s)) {
            ESP_LOGI(TAG, "UDP connection successful");
            return true;
        } else {
            ESP_LOGE(TAG, "UDP connnection failed");
            return false;
        }
    } else if (modem.module == LBAD0XX1SC) {
        #if MODEM_TYPE == CELLULAR
            sendAT("AT%SOCKETCMD=\"ALLOCATE\",1,\"UDP\",\"OPEN\",\"142.11.236.169\",5255\r", "OK", network_timeout_s);
        #elif MODEM_TYPE == SATELLITE
            sendAT("AT%SOCKETCMD=\"ALLOCATE\",1,\"UDP\",\"OPEN\",\"142.11.236.169\",5255\r", "OK", network_timeout_s);
        #endif
        if (sendAT("AT%SOCKETCMD=\"ACTIVATE\",1\r", "OK", connect_timeout_s)) {
            ESP_LOGI(TAG, "UDP connection successful");
            return true;
        } else {
            ESP_LOGE(TAG, "UDP connnection failed");
            return false;
        }
    } else {
        return false;
    }
}

bool send_UDP(const uint8_t *data, const uint8_t len)
{
    const uint8_t send_timeout_s = 20;
    if (modem.module == SIM7070) {
        char pub_buf[32] = "AT+CASEND=1,";
        char msg_len[32];
        sprintf(msg_len, "%d", len);
        strcat(pub_buf, msg_len);
        strcat(pub_buf, "\r");
        sendAT(pub_buf, ">", 5);
        if (send_bytes(data, len, "+CAURC: \"recv\"", send_timeout_s)) {
            ESP_LOGI(TAG, "UDP send successful");
            return true;
        } else {
            ESP_LOGE(TAG, "UDP send failed");
            return false;
        }
    } else if (modem.module == A7670) {
        char pub_buf[64] = "AT+CIPSEND=1,";
        char msg_len[32];
        sprintf(msg_len, "%d", len);
        strncat(pub_buf, msg_len, strlen(msg_len));
        strncat(pub_buf, ",\"142.11.236.169\",5254\r", strlen(",\"142.11.236.169\",5254\r") + 1);
        sendAT(pub_buf, ">", 5);
        
        if(sendAT((char *)data, "RECV FROM:142.11.236.169", send_timeout_s)) {
            ESP_LOGI(TAG, "UDP send successful");
            return true;
        } else {
            ESP_LOGE(TAG, "UDP send failed");
            return false;
        }
    } else if (modem.module == LBAD0XX1SC) {
        char pub_buf[64] = "AT%SOCKETDATA=\"SEND\",1,";
        char msg_len[32];
        sprintf(msg_len, "%d", strlen((char *)data)/2);
        strcat(pub_buf, msg_len);
        strcat(pub_buf, ",\"");
        strcat(pub_buf, (char *)data);
        strcat(pub_buf, "\",,,1\r");
        if(sendAT(pub_buf, "SOCKETEV:5,1", send_timeout_s)) {
            ESP_LOGI(TAG, "UDP send successful");
            return true;
        } else {
            ESP_LOGE(TAG, "UDP send failed");
            return false;
        }
    } else {
        return false;
    }
}

bool transmit_data(const uint8_t *data)
{
    bool transmit_success = false;

    xEventGroupClearBits(event_group, PROCEED_BIT); // Clear AT response signal
    xEventGroupClearBits(event_group, ERROR_BIT); // Clear AT error response signal
    uart_init();
    xTaskCreate(rx_task, "uart_rx_task", RX_BUF_SIZE * 2, NULL, configMAX_PRIORITIES - 1, NULL); // Start AT response receiver task
    if (modem.module == SIM7070){
        AT_response = "SMS Ready"; // Set modem ready signal
    } else if (modem.module == A7670){
        AT_response = "ATREADY"; // Set modem ready signal
    } else if (modem.module == LBAD0XX1SC){
        AT_response = "BOOTEV"; // Set modem ready signal
    }
    modem_power_on();
    xEventGroupWaitBits(event_group, PROCEED_BIT, pdTRUE, pdTRUE, 30000 / portTICK_PERIOD_MS); // Wait for modem ready signal

    #if MODEM_TYPE == SATELLITE
        EventBits_t uxBits;
        if (modem.sleep_mode == PSM){
            if (sendAT("AT+CREG?\r", "OK", 1)){
                uxBits = 1;
            } else {
                uxBits = 1;
            }
        } else {
            sendAT("AT+CFUN=0\r", "OK", 5); // Turn off radio
            sendAT("AT%RATACT=\"NBNTN\"\r", "OK", 10); // Enable NB-NTN
            sendAT("AT%IGNSSEV=\"FIX\",1\r", "OK", 1); // Enable GPS fix reporting
            sendAT("AT+CEREG=2\r", "OK", 1); // Enable network registration reporting
            sendAT("AT%IGNSSACT=1\r", "OK", 1); // Start GNSS

            AT_response = "FIX";
            xEventGroupWaitBits(event_group, PROCEED_BIT, pdTRUE, pdTRUE, portMAX_DELAY); // Wait for GPS fix
            sendAT("AT%IGNSSINFO=\"FIX\"\r", "%IGNSSINFO:", 1); // Get GPS info
            sendAT("AT%IGNSSACT=0\r", "OK", 1); // Stop GNSS
            if(sendAT("AT+CFUN=1\r", "CEREG: 5", 180)){ // Turn on radio
                uxBits = 1;
            } else {
                uxBits = 0;
            }
        }
    #else
        if (modem.module == A7670) {
            sendAT("AT+CTZR=1\r", "OK", 1); // Enable time zone reporting
        } else if (modem.module == LBAD0XX1SC) {
            sendAT("AT%NOTIFYEV=\"LTIME\",1\r", "OK", 1);
        }
        if (modem.module == SIM7070) {
            if (modem.sleep_mode == PSM) {
                sendAT("AT+CPSMS=0\r", "OK", 1); // Disable PSM
            }
            sendAT("AT+CEREG=1\r", "OK", 1);
            AT_response = "CEREG: 5";
        } else if (modem.module == A7670) {
            AT_response = "CTZV";
            //AT_response = "CGEV: EPS PDN ACT 1";
        } else if (modem.module == LBAD0XX1SC) {
            AT_response = "LTIME";
        }   
        EventBits_t uxBits;
        uxBits = xEventGroupWaitBits(event_group, PROCEED_BIT, pdTRUE, pdTRUE, 30000 / portTICK_PERIOD_MS); // Wait for network attachment
    #endif

    if ((uxBits & PROCEED_BIT) != 0) { // Received correct response
        ESP_LOGI(TAG, "Attached to network");
        if (connect_UDP()) {
            if (queued) { //Publish any queued messages first
                if (mount_file_system()) {
                    read_file("/sdcard/queue.txt");
                    unlink("/sdcard/queue.txt");
                    rename("/sdcard/failed.txt", "/sdcard/queue.txt");
                    unmount_file_system();
                }
            }
            if (!queued) {
                if (send_UDP(data, 35)) {
                    transmit_success = true;
                }
            }
            if (modem.module == SIM7070) {
                sendAT("AT+CNACT=0,0\r", "OK", 1);
            } else if (modem.module == LBAD0XX1SC) {
                sendAT("AT%SOCKETCMD=\"DELETE\",1\r", "OK", 1);
            }
        }
    } else {
        ESP_LOGE(TAG, "Could not attach to network");
    }

    if (modem.sleep_mode == PSM) {
        ESP_LOGI(TAG, "Modem entering PSM...");
        sendAT("AT+CPSMS=1\r", "OK", 1);
    } else {
        modem_power_off();
    }
    
    return transmit_success;
}