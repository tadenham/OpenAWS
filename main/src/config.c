#include "station.h"
#include "modem.h"

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define BLE_GAP_APPEARANCE_GENERIC_TAG 0x0200
#define BLE_GAP_LE_ROLE_PERIPHERAL 0x00
#define BLE_GAP_URI_PREFIX_HTTPS 0x17

static void start_advertising(void);
static int gap_event_handler(struct ble_gap_event *event, void *arg);
void ble_store_config_init(void);

static int station_id_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int station_name_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int station_user_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int time_zone_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int battery_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int temp_sensor_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

static int module_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int network_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int apn_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int operator_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

static void send_station_id_indication();
static void send_station_name_indication();
static void send_station_user_indication();
static void send_time_zone_indication();
static void send_battery_indication();
static void send_temp_sensor_indication();

static void send_module_indication();
static void send_network_indication();
static void send_apn_indication();
static void send_operator_indication();

static void set_apn();

static nvs_handle_t handle;

static const uint8_t esp_uri[] = {BLE_GAP_URI_PREFIX_HTTPS, '/', '/', 'o', 'p', 'e', 'n', 'a', 'w', 's', '.', 'o', 'r', 'g'};
static uint8_t addr_val[6] = {0};
static uint8_t own_addr_type;

static const ble_uuid16_t device_info_svc_uuid = BLE_UUID16_INIT(0x180A); // Device information service

static const ble_uuid128_t station_id_chr_uuid = BLE_UUID128_INIT(0x3b, 0xe0, 0xc8, 0x9e, 0xf9, 0xfe, 0xf7, 0x99, 0x4f, 0x4e, 0xb0, 0xcc, 0xb0, 0x48, 0x66, 0x85);
static const ble_uuid128_t station_name_chr_uuid = BLE_UUID128_INIT(0x2e, 0xde, 0x14, 0x79, 0xbd, 0x15, 0x76, 0x83, 0xc8, 0x49, 0x99, 0xc0, 0x55, 0x59, 0x19, 0x09);
static const ble_uuid128_t station_user_chr_uuid = BLE_UUID128_INIT(0x70, 0x62, 0xc8, 0x82, 0x45, 0x35, 0x5d, 0xa8, 0x8a, 0x49, 0xe7, 0x08, 0x1b, 0xf6, 0x46, 0xb1);
static const ble_uuid128_t time_zone_chr_uuid = BLE_UUID128_INIT(0x86, 0x78, 0x8c, 0x66, 0x23, 0x48, 0x28, 0xad, 0x0a, 0x4e, 0xcc, 0xb1, 0xa4, 0x48, 0x5f, 0x26);
static const ble_uuid128_t battery_chr_uuid = BLE_UUID128_INIT(0x1a, 0x9e, 0xb8, 0x35, 0x47, 0x04, 0x9b, 0xaa, 0xe6, 0x48, 0x08, 0xa6, 0xe7, 0x43, 0x38, 0xe9);
static const ble_uuid128_t temp_sensor_chr_uuid = BLE_UUID128_INIT(0x1d, 0x80, 0x11, 0xef, 0xb7, 0x73, 0x88, 0x89, 0x8f, 0x4e, 0x5b, 0x39, 0x14, 0xb6, 0xfb, 0x38);
static const ble_uuid128_t network_chr_uuid = BLE_UUID128_INIT(0x8d, 0x4e, 0x94, 0x68, 0xc5, 0x28, 0xf8, 0xb8, 0x43, 0x41, 0x92, 0xbc, 0xe0, 0x56, 0xe3, 0xd6);
static const ble_uuid128_t module_chr_uuid = BLE_UUID128_INIT(0xcf, 0xea, 0x2b, 0x84, 0xf4, 0xa8, 0x2f, 0x96, 0x26, 0x41, 0x12, 0xef, 0x34, 0x4a, 0x91, 0xf1);
static const ble_uuid128_t apn_chr_uuid = BLE_UUID128_INIT(0x82, 0x5c, 0xd8, 0x1f, 0x51, 0x5b, 0xf2, 0x8b, 0x47, 0x4e, 0xbe, 0xfb, 0x6c, 0xf2, 0xb8, 0x20);
static const ble_uuid128_t operator_chr_uuid = BLE_UUID128_INIT(0x07, 0x56, 0xf8, 0xf9, 0x89, 0x4f, 0x76, 0x8e, 0xba, 0x40, 0xcd, 0xc7, 0x58, 0x97, 0xbd, 0xc8);

static uint8_t station_id_chr_val[6] = {0};
static uint8_t station_name_chr_val[32] = {0};
static uint8_t station_user_chr_val[32] = {0};
static uint8_t time_zone_chr_val[48] = {0};

static uint8_t apn_chr_val[32] = {0};
static uint8_t operator_chr_val[32] = {0};

static uint16_t station_id_chr_val_handle;
static uint16_t station_name_chr_val_handle;
static uint16_t station_user_chr_val_handle;
static uint16_t time_zone_chr_val_handle;
static uint16_t battery_chr_val_handle;     
static uint16_t temp_sensor_chr_val_handle;

static uint16_t module_chr_val_handle;
static uint16_t network_chr_val_handle;
static uint16_t apn_chr_val_handle;
static uint16_t operator_chr_val_handle;

static uint16_t station_id_chr_conn_handle = 0;
static uint16_t station_name_chr_conn_handle = 0;
static uint16_t station_user_chr_conn_handle = 0;
static uint16_t time_zone_chr_conn_handle = 0;
static uint16_t battery_chr_conn_handle = 0;
static uint16_t temp_sensor_chr_conn_handle = 0;

static uint16_t module_chr_conn_handle = 0;
static uint16_t network_chr_conn_handle = 0;
static uint16_t apn_chr_conn_handle = 0;
static uint16_t operator_chr_conn_handle = 0;

static bool station_id_chr_conn_handle_inited = false;
static bool station_name_chr_conn_handle_inited = false;
static bool station_user_chr_conn_handle_inited = false;
static bool time_zone_chr_conn_handle_inited = false;
static bool battery_chr_conn_handle_inited = false;
static bool temp_sensor_chr_conn_handle_inited = false;

static bool module_chr_conn_handle_inited = false;
static bool network_chr_conn_handle_inited = false;
static bool apn_chr_conn_handle_inited = false;
static bool operator_chr_conn_handle_inited = false;

static bool station_id_ind_status = false;
static bool station_name_ind_status = false;
static bool station_user_ind_status = false;
static bool time_zone_ind_status = false;
static bool battery_ind_status = false;
static bool temp_sensor_ind_status = false;

static bool module_ind_status = false;
static bool network_ind_status = false;
static bool apn_ind_status = false;
static bool operator_ind_status = false;

// GATT services table
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    // Device information service
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &device_info_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            { // Station ID characteristic
                .uuid = &station_id_chr_uuid.u,
                .access_cb = station_id_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE, // Read/indicate
                .val_handle = &station_id_chr_val_handle
            },
            { // Station name characteristic
                .uuid = &station_name_chr_uuid.u,
                .access_cb = station_name_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_WRITE, // Read/indicate/write
                .val_handle = &station_name_chr_val_handle
            },
            { // Station user characteristic
                .uuid = &station_user_chr_uuid.u,
                .access_cb = station_user_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_WRITE, // Read/indicate/write
                .val_handle = &station_user_chr_val_handle
            },
            { // Time zone characteristic
                .uuid = &time_zone_chr_uuid.u,
                .access_cb = time_zone_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_WRITE, // Read/indicate/write
                .val_handle = &time_zone_chr_val_handle
            },
            { // Battery characteristic
                .uuid = &battery_chr_uuid.u,
                .access_cb = battery_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_WRITE, // Read/indicate/write
                .val_handle = &battery_chr_val_handle
            },
            { // Temperature sensor characteristic
                .uuid = &temp_sensor_chr_uuid.u,
                .access_cb = temp_sensor_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE, // Read/indicate
                .val_handle = &temp_sensor_chr_val_handle
            },
            { // Module characteristic
                .uuid = &module_chr_uuid.u,
                .access_cb = module_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE, // Read/indicate
                .val_handle = &module_chr_val_handle
            },
            { // Network characteristic
                .uuid = &network_chr_uuid.u,
                .access_cb = network_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_WRITE, // Read/indicate/write
                .val_handle = &network_chr_val_handle
            },
            { // APN characteristic
                .uuid = &apn_chr_uuid.u,
                .access_cb = apn_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_WRITE, // Read/indicate/write
                .val_handle = &apn_chr_val_handle
            },
            { // Operator characteristic
                .uuid = &operator_chr_uuid.u,
                .access_cb = operator_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE, // Read/indicate
                .val_handle = &operator_chr_val_handle
            },
            {
                0, // No more characteristics in service
            }
        }
    },
    {
        0, // No more services
    },
};

static int gap_init(void)
{
    int rc = 0;
    ble_svc_gap_init(); // Call NimBLE GAP initialization API
    rc = ble_svc_gap_device_name_set(DEVICE_NAME); // Set GAP device name
    if (rc != 0){
        ESP_LOGE(TAG, "failed to set device name to %s, error code: %d", DEVICE_NAME, rc);
        return rc;
    }
    return rc;
}

/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
static int gatt_svc_init(void)
{
    int rc;
    ble_svc_gatt_init(); // GATT service initialization
    rc = ble_gatts_count_cfg(gatt_svr_svcs); // Update GATT services counter
    if (rc != 0){
        return rc;
    }
    rc = ble_gatts_add_svcs(gatt_svr_svcs); // Add GATT services
    if (rc != 0){
        return rc;
    }
    return 0;
}

static int station_id_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    switch (ctxt->op){ // Handle access events
        case BLE_GATT_ACCESS_OP_READ_CHR: // Read characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){ // Verify connection handle
                ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == station_id_chr_val_handle){ // Verify attribute handle
                rc = os_mbuf_append(ctxt->om, &station_id_chr_val, sizeof(station_id_chr_val));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            goto error;
        default: // Unknown event
            goto error;
    }
    error:
        ESP_LOGE(TAG, "unexpected access operation to station ID characteristic, opcode: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
}

static int station_name_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op){ // Handle access events
        case BLE_GATT_ACCESS_OP_READ_CHR: // Read characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){ // Verify connection handle
                ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == station_name_chr_val_handle){ // Verify attribute handle
                rc = os_mbuf_append(ctxt->om, &station_name_chr_val, sizeof(station_name_chr_val));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            goto error;
        case BLE_GATT_ACCESS_OP_WRITE_CHR: // Write characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){
                ESP_LOGI(TAG, "characteristic write; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic write by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == station_name_chr_val_handle){ // Verify attribute handle
                if (ctxt->om->om_len){
                    memset(station_name_chr_val, 0, sizeof(station_name_chr_val)); // Clear station name value
                    memcpy(station_name_chr_val, ctxt->om->om_data, ctxt->om->om_len); // Update station name value
                    ESP_LOGI(TAG, "Station name updated to %s", station_name_chr_val);
                    nvs_set_str(handle, "station_name", (char *)station_name_chr_val);
                    esp_err_t ret = nvs_commit(handle);
                    if (ret != ESP_OK){
                        ESP_LOGE(TAG, "Failed to commit data");
                    } else {
                        ESP_LOGI(TAG, "Station name saved to NVS");
                    }
                    send_station_name_indication();
                } else {
                    goto error;
                }
                return rc;
            }
            goto error;
        default: // Unknown event
            goto error;
    }
    error:
        ESP_LOGE(TAG, "unexpected access operation to station name characteristic, opcode: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
}

static int station_user_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op){ // Handle access events
        case BLE_GATT_ACCESS_OP_READ_CHR: // Read characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){ // Verify connection handle
                ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == station_user_chr_val_handle){ // Verify attribute handle
                rc = os_mbuf_append(ctxt->om, &station_user_chr_val, sizeof(station_user_chr_val));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            goto error;
        case BLE_GATT_ACCESS_OP_WRITE_CHR: // Write characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){
                ESP_LOGI(TAG, "characteristic write; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic write by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == station_user_chr_val_handle){ // Verify attribute handle
                if (ctxt->om->om_len){
                    memset(station_user_chr_val, 0, sizeof(station_user_chr_val)); // Clear station user value
                    memcpy(station_user_chr_val, ctxt->om->om_data, ctxt->om->om_len); // Update station user value
                    ESP_LOGI(TAG, "Station user updated to %s", station_user_chr_val);
                    nvs_set_str(handle, "station_user", (char *)station_user_chr_val);
                    esp_err_t ret = nvs_commit(handle);
                    if (ret != ESP_OK){
                        ESP_LOGE(TAG, "Failed to commit data");
                    } else {
                        ESP_LOGI(TAG, "Station user saved to NVS");
                    }
                } else {
                    goto error;
                }
                return rc;
            }
            goto error;
        default: // Unknown event
            goto error;
    }
    error:
        ESP_LOGE(TAG, "unexpected access operation to station user characteristic, opcode: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
}

static int time_zone_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op){ // Handle access events
        case BLE_GATT_ACCESS_OP_READ_CHR: // Read characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){ // Verify connection handle
                ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == time_zone_chr_val_handle){ // Verify attribute handle
                rc = os_mbuf_append(ctxt->om, &time_zone_chr_val, sizeof(time_zone_chr_val));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            goto error;
        case BLE_GATT_ACCESS_OP_WRITE_CHR: // Write characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){
                ESP_LOGI(TAG, "characteristic write; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic write by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == time_zone_chr_val_handle){ // Verify attribute handle
                if (ctxt->om->om_len){
                    memset(time_zone_chr_val, 0, sizeof(time_zone_chr_val)); // Clear time zone value
                    memcpy(time_zone_chr_val, ctxt->om->om_data, ctxt->om->om_len); // Update time zone value
                    strcpy(station.time_zone, (char *)time_zone_chr_val);
                    ESP_LOGI(TAG, "Time zone updated to %s", time_zone_chr_val);
                    nvs_set_str(handle, "time_zone", (char *)time_zone_chr_val);
                    esp_err_t ret = nvs_commit(handle);
                    if (ret != ESP_OK){
                        ESP_LOGE(TAG, "Failed to commit data");
                    } else {
                        ESP_LOGI(TAG, "Time zone saved to NVS");
                    }
                    send_time_zone_indication();
                } else {
                    goto error;
                }
                return rc;
            }
            goto error;
        default: // Unknown event
            goto error;
    }
    error:
        ESP_LOGE(TAG, "unexpected access operation to time zone characteristic, opcode: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
}

static int battery_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op){ // Handle access events
        case BLE_GATT_ACCESS_OP_READ_CHR: // Read characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){ // Verify connection handle
                ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == battery_chr_val_handle){ // Verify attribute handle
                rc = os_mbuf_append(ctxt->om, &station.battery_type, sizeof(station.battery_type));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            goto error;
        case BLE_GATT_ACCESS_OP_WRITE_CHR: // Write characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){
                ESP_LOGI(TAG, "characteristic write; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic write by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == battery_chr_val_handle){ // Verify attribute handle
                if (ctxt->om->om_len){
                    station.battery_type = ctxt->om->om_data[0]; // Update battery type value
                    ESP_LOGI(TAG, "Battery type updated to %d", station.battery_type);
                    nvs_set_u8(handle, "battery_type", station.battery_type);
                    esp_err_t ret = nvs_commit(handle);
                    if (ret != ESP_OK){
                        ESP_LOGE(TAG, "Failed to commit data");
                    } else {
                        ESP_LOGI(TAG, "Battery type saved to NVS");
                    }
                    send_battery_indication();
                } else {
                    goto error;
                }
                return rc;
            }
            goto error;
        default: // Unknown event
            goto error;
    }
    error:
        ESP_LOGE(TAG, "unexpected access operation to battery type characteristic, opcode: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
}

static int temp_sensor_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op){ // Handle access events
        case BLE_GATT_ACCESS_OP_READ_CHR: // Read characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){ // Verify connection handle
                ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == temp_sensor_chr_val_handle){ // Verify attribute handle
                rc = os_mbuf_append(ctxt->om, &station.temp_sensor, sizeof(station.temp_sensor));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            goto error;
        default: // Unknown event
            goto error;
    }
    error:
        ESP_LOGE(TAG, "unexpected access operation to temperature sensor characteristic, opcode: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
}

static int module_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op){ // Handle access events
        case BLE_GATT_ACCESS_OP_READ_CHR: // Read characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){ // Verify connection handle
                ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == module_chr_val_handle){ // Verify attribute handle
                rc = os_mbuf_append(ctxt->om, &modem.module, sizeof(modem.module));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            goto error;
        default: // Unknown event
            goto error;
    }
    error:
        ESP_LOGE(TAG, "unexpected access operation to module characteristic, opcode: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
}

static int network_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op){ // Handle access events
        case BLE_GATT_ACCESS_OP_READ_CHR: // Read characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE) { // Verify connection handle
                ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == network_chr_val_handle) { // Verify attribute handle
                rc = os_mbuf_append(ctxt->om, &modem.network, sizeof(modem.network));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            goto error;
        case BLE_GATT_ACCESS_OP_WRITE_CHR: // Write characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){
                ESP_LOGI(TAG, "characteristic write; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic write by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == network_chr_val_handle) { // Verify attribute handle
                if (ctxt->om->om_len){
                    modem.network = ctxt->om->om_data[0]; // Update network type value
                    ESP_LOGI(TAG, "Network type updated to %d", modem.network);
                    nvs_set_u8(handle, "network_type", modem.network);
                    esp_err_t ret = nvs_commit(handle);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to commit data");
                    } else {
                        ESP_LOGI(TAG, "Network type saved to NVS");
                    }
                    send_network_indication();
                } else {
                    goto error;
                }
                return rc;
            }
            goto error;
        default: // Unknown event
            goto error;
    }
    error:
        ESP_LOGE(TAG, "unexpected access operation to network type characteristic, opcode: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
}

static int apn_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op){ // Handle access events
        case BLE_GATT_ACCESS_OP_READ_CHR: // Read characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){ // Verify connection handle
                ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == apn_chr_val_handle){ // Verify attribute handle
                rc = os_mbuf_append(ctxt->om, &apn_chr_val, sizeof(apn_chr_val));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            goto error;
        case BLE_GATT_ACCESS_OP_WRITE_CHR: // Write characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){
                ESP_LOGI(TAG, "characteristic write; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic write by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == apn_chr_val_handle){ // Verify attribute handle
                if (ctxt->om->om_len){
                    memset(apn_chr_val, 0, sizeof(apn_chr_val)); // Clear APN value
                    memcpy(apn_chr_val, ctxt->om->om_data, ctxt->om->om_len); // Update APN value
                    modem.apn = (char *)apn_chr_val;
                    ESP_LOGI(TAG, "APN updated to %s", modem.apn);
                    nvs_set_str(handle, "apn", modem.apn);
                    esp_err_t ret = nvs_commit(handle);
                    if (ret != ESP_OK){
                        ESP_LOGE(TAG, "Failed to commit data");
                    } else {
                        ESP_LOGI(TAG, "APN saved to NVS");
                    }
                    set_apn();
                } else {
                    goto error;
                }
                return rc;
            }
            goto error;
        default: // Unknown event
            goto error;
    }
    error:
        ESP_LOGE(TAG, "unexpected access operation to APN characteristic, opcode: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
}

static int operator_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;
    switch (ctxt->op){ // Handle access events
        case BLE_GATT_ACCESS_OP_READ_CHR: // Read characteristic event
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE){ // Verify connection handle
                ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
            } else {
                //ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d", attr_handle);
            }
            if (attr_handle == operator_chr_val_handle){ // Verify attribute handle
                rc = os_mbuf_append(ctxt->om, &operator_chr_val, sizeof(operator_chr_val));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            goto error;
        default: // Unknown event
            goto error;
    }
    error:
        ESP_LOGE(TAG, "unexpected access operation to operator characteristic, opcode: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
}

static void send_station_id_indication(void)
{
    if (station_id_ind_status && station_id_chr_conn_handle_inited){
        esp_efuse_mac_get_default(station_id_chr_val);
        ble_gatts_indicate(station_id_chr_conn_handle, station_id_chr_val_handle);
        ESP_LOGI(TAG, "Station ID indication sent!");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void send_station_name_indication(void)
{
    if (station_name_ind_status && station_name_chr_conn_handle_inited){
        ble_gatts_indicate(station_name_chr_conn_handle, station_name_chr_val_handle);
        ESP_LOGI(TAG, "Station name indication sent!");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }  
}

static void send_station_user_indication(void)
{
    if (station_user_ind_status && station_user_chr_conn_handle_inited){
        ble_gatts_indicate(station_user_chr_conn_handle, station_user_chr_val_handle);
        ESP_LOGI(TAG, "Station user indication sent!");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void send_time_zone_indication(void)
{
    if (time_zone_ind_status && time_zone_chr_conn_handle_inited) {
        ble_gatts_indicate(time_zone_chr_conn_handle, time_zone_chr_val_handle);
        ESP_LOGI(TAG, "Time zone indication sent!");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void send_battery_indication(void)
{
    if (battery_ind_status && battery_chr_conn_handle_inited) {
        ble_gatts_indicate(battery_chr_conn_handle, battery_chr_val_handle);
        ESP_LOGI(TAG, "Battery indication sent!");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void send_temp_sensor_indication(void)
{
    if (temp_sensor_ind_status && temp_sensor_chr_conn_handle_inited) {
        ble_gatts_indicate(temp_sensor_chr_conn_handle, temp_sensor_chr_val_handle);
        ESP_LOGI(TAG, "Temperature sensor indication sent!");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void send_module_indication(void)
{
    if (module_ind_status && module_chr_conn_handle_inited) {
        ble_gatts_indicate(module_chr_conn_handle, module_chr_val_handle);
        ESP_LOGI(TAG, "Module indication sent!");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    } 
}

static void send_network_indication(void)
{
    if (network_ind_status && network_chr_conn_handle_inited) {
        ble_gatts_indicate(network_chr_conn_handle, network_chr_val_handle);
        ESP_LOGI(TAG, "Network indication sent!");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void send_apn_indication(void)
{
    if (apn_ind_status && apn_chr_conn_handle_inited) {
        ble_gatts_indicate(apn_chr_conn_handle, apn_chr_val_handle);
        ESP_LOGI(TAG, "APN indication sent!");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void send_operator_indication(void)
{
    if (operator_ind_status && operator_chr_conn_handle_inited) {
        ble_gatts_indicate(operator_chr_conn_handle, operator_chr_val_handle);
        ESP_LOGI(TAG, "Operator indication sent!");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    /* Local variables */
    char buf[BLE_UUID_STR_LEN];

    /* Handle GATT attributes register events */
    switch (ctxt->op) {

    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG,
                 "registering characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;

    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registering descriptor %s with handle=%d", ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
        break;

    /* Unknown event */
    default:
        assert(0);
        break;
    }
}

/*
 *  GATT server subscribe event callback
 *   
 */

static void gatt_svr_subscribe_cb(struct ble_gap_event *event)
{
    /* Check connection handle */
    if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle);
    } else {
        ESP_LOGI(TAG, "subscribe by nimble stack; attr_handle=%d",
                 event->subscribe.attr_handle);
    }

    /* Check attribute handle */
    if (event->subscribe.attr_handle == station_id_chr_val_handle) {
        /* Update station ID subscription status */
        station_id_chr_conn_handle = event->subscribe.conn_handle;
        station_id_chr_conn_handle_inited = true;
        station_id_ind_status = event->subscribe.cur_indicate;
        //send_station_id_indication();
    }
    if (event->subscribe.attr_handle == station_name_chr_val_handle) {
        /* Update station name subscription status */
        station_name_chr_conn_handle = event->subscribe.conn_handle;
        station_name_chr_conn_handle_inited = true;
        station_name_ind_status = event->subscribe.cur_indicate;
        //send_station_name_indication();
    }
    if (event->subscribe.attr_handle == station_user_chr_val_handle) {
        /* Update station user subscription status */
        station_user_chr_conn_handle = event->subscribe.conn_handle;
        station_user_chr_conn_handle_inited = true;
        station_user_ind_status = event->subscribe.cur_indicate;
        //send_station_user_indication();
    }
    if (event->subscribe.attr_handle == time_zone_chr_val_handle) {
        /* Update time zone subscription status */
        time_zone_chr_conn_handle = event->subscribe.conn_handle;
        time_zone_chr_conn_handle_inited = true;
        time_zone_ind_status = event->subscribe.cur_indicate;
        //send_time_zone_indication();
    }
    if (event->subscribe.attr_handle == battery_chr_val_handle) {
        /* Update battery subscription status */
        battery_chr_conn_handle = event->subscribe.conn_handle;
        battery_chr_conn_handle_inited = true;
        battery_ind_status = event->subscribe.cur_indicate;
        //send_battery_indication();
    }
    if (event->subscribe.attr_handle == temp_sensor_chr_val_handle) {
        /* Update temperature sensor subscription status */
        temp_sensor_chr_conn_handle = event->subscribe.conn_handle;
        temp_sensor_chr_conn_handle_inited = true;
        temp_sensor_ind_status = event->subscribe.cur_indicate;
        //send_temp_sensor_indication();
    }
    if (event->subscribe.attr_handle == module_chr_val_handle) {
        /* Update module subscription status */
        module_chr_conn_handle = event->subscribe.conn_handle;
        module_chr_conn_handle_inited = true;
        module_ind_status = event->subscribe.cur_indicate;
        //send_module_indication();
    }
    if (event->subscribe.attr_handle == network_chr_val_handle) {
        /* Update network subscription status */
        network_chr_conn_handle = event->subscribe.conn_handle;
        network_chr_conn_handle_inited = true;
        network_ind_status = event->subscribe.cur_indicate;
        //send_network_indication();
    }
    if (event->subscribe.attr_handle == apn_chr_val_handle) {
        /* Update APN subscription status */
        apn_chr_conn_handle = event->subscribe.conn_handle;
        apn_chr_conn_handle_inited = true;
        apn_ind_status = event->subscribe.cur_indicate;
        //send_apn_indication();
    }
    if (event->subscribe.attr_handle == operator_chr_val_handle) {
        /* Update operator subscription status */
        operator_chr_conn_handle = event->subscribe.conn_handle;
        operator_chr_conn_handle_inited = true;
        operator_ind_status = event->subscribe.cur_indicate;
        //send_operator_indication();
    }
}

inline static void format_addr(char *addr_str, uint8_t addr[])
{
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1],
            addr[2], addr[3], addr[4], addr[5]);
}

static void print_conn_desc(struct ble_gap_conn_desc *desc)
{
    /* Local variables */
    char addr_str[18] = {0};

    /* Connection handle */
    ESP_LOGI(TAG, "connection handle: %d", desc->conn_handle);

    /* Local ID address */
    format_addr(addr_str, desc->our_id_addr.val);
    ESP_LOGI(TAG, "device id address: type=%d, value=%s",
             desc->our_id_addr.type, addr_str);

    /* Peer ID address */
    format_addr(addr_str, desc->peer_id_addr.val);
    ESP_LOGI(TAG, "peer id address: type=%d, value=%s", desc->peer_id_addr.type,
             addr_str);

    /* Connection info */
    ESP_LOGI(TAG,
             "conn_itvl=%d, conn_latency=%d, supervision_timeout=%d, "
             "encrypted=%d, authenticated=%d, bonded=%d\n",
             desc->conn_itvl, desc->conn_latency, desc->supervision_timeout,
             desc->sec_state.encrypted, desc->sec_state.authenticated,
             desc->sec_state.bonded);
}

static void start_advertising(void)
{
    /* Local variables */
    int rc = 0;
    const char *name;
    struct ble_hs_adv_fields adv_fields = {0};
    struct ble_hs_adv_fields rsp_fields = {0};
    struct ble_gap_adv_params adv_params = {0};

    /* Set advertising flags */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Set device name */
    name = ble_svc_gap_device_name();
    adv_fields.name = (uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;

    /* Set device tx power */
    adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    adv_fields.tx_pwr_lvl_is_present = 1;

    /* Set device appearance */
    adv_fields.appearance = BLE_GAP_APPEARANCE_GENERIC_TAG;
    adv_fields.appearance_is_present = 1;

    /* Set device LE role */
    adv_fields.le_role = BLE_GAP_LE_ROLE_PERIPHERAL;
    adv_fields.le_role_is_present = 1;

    /* Set advertiement fields */
    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set advertising data, error code: %d", rc);
        return;
    }

    /* Set device address */
    rsp_fields.device_addr = addr_val;
    rsp_fields.device_addr_type = own_addr_type;
    rsp_fields.device_addr_is_present = 1;

    /* Set URI */
    rsp_fields.uri = esp_uri;
    rsp_fields.uri_len = sizeof(esp_uri);

    /* Set advertising interval */
    rsp_fields.adv_itvl = BLE_GAP_ADV_ITVL_MS(500);
    rsp_fields.adv_itvl_is_present = 1;

    /* Set scan response fields */
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set scan response data, error code: %d", rc);
        return;
    }

    /* Set non-connetable and general discoverable mode to be a beacon */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    /* Set advertising interval */
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(510);

    /* Start advertising */
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                           gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to start advertising, error code: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "advertising started!");
}

/*
 * NimBLE applies an event-driven model to keep GAP service going
 * gap_event_handler is a callback function registered when calling
 * ble_gap_adv_start API and called when a GAP event arrives
 */
static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    /* Local variables */
    int rc = 0;
    struct ble_gap_conn_desc desc;

    /* Handle different GAP event */
    switch (event->type) {

    /* Connect event */
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        ESP_LOGI(TAG, "connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);

        /* Connection succeeded */
        if (event->connect.status == 0) {
            /* Check connection handle */
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc != 0) {
                ESP_LOGE(TAG,
                         "failed to find connection by handle, error code: %d",
                         rc);
                return rc;
            }

            /* Print connection descriptor */
            print_conn_desc(&desc);

            /* Try to update connection parameters */
            struct ble_gap_upd_params params = {.itvl_min = desc.conn_itvl,
                                                .itvl_max = desc.conn_itvl,
                                                .latency = 3,
                                                .supervision_timeout =
                                                    desc.supervision_timeout};
            rc = ble_gap_update_params(event->connect.conn_handle, &params);
            if (rc != 0) {
                ESP_LOGE(
                    TAG,
                    "failed to update connection parameters, error code: %d",
                    rc);
                return rc;
            }
        }
        /* Connection failed, restart advertising */
        else {
            start_advertising();
        }
        return rc;

    /* Disconnect event */
    case BLE_GAP_EVENT_DISCONNECT:
        /* A connection was terminated, print connection descriptor */
        ESP_LOGI(TAG, "disconnected from peer; reason=%d",
                 event->disconnect.reason);

        /* Restart advertising */
        //start_advertising();
        xEventGroupSetBits(event_group, SETUP_DONE_BIT);
        return rc;

    /* Connection parameters update event */
    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        ESP_LOGI(TAG, "connection updated; status=%d",
                 event->conn_update.status);

        /* Print connection descriptor */
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (rc != 0) {
            ESP_LOGE(TAG, "failed to find connection by handle, error code: %d",
                     rc);
            return rc;
        }
        print_conn_desc(&desc);
        return rc;

    /* Advertising complete event */
    case BLE_GAP_EVENT_ADV_COMPLETE:
        /* Advertising completed, restart advertising */
        ESP_LOGI(TAG, "advertise complete; reason=%d",
                 event->adv_complete.reason);
        start_advertising();
        return rc;

    /* Notification sent event */
    case BLE_GAP_EVENT_NOTIFY_TX:
        if ((event->notify_tx.status != 0) &&
            (event->notify_tx.status != BLE_HS_EDONE)) {
            /* Print notification info on error */
            ESP_LOGI(TAG,
                     "notify event; conn_handle=%d attr_handle=%d "
                     "status=%d is_indication=%d",
                     event->notify_tx.conn_handle, event->notify_tx.attr_handle,
                     event->notify_tx.status, event->notify_tx.indication);
        }
        return rc;

    /* Subscribe event */
    case BLE_GAP_EVENT_SUBSCRIBE:
        /* Print subscription info to log */
        ESP_LOGI(TAG,
                 "subscribe event; conn_handle=%d attr_handle=%d "
                 "reason=%d prevn=%d curn=%d previ=%d curi=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle,
                 event->subscribe.reason, event->subscribe.prev_notify,
                 event->subscribe.cur_notify, event->subscribe.prev_indicate,
                 event->subscribe.cur_indicate);

        /* GATT subscribe event callback */
        gatt_svr_subscribe_cb(event);
        return rc;

    /* MTU update event */
    case BLE_GAP_EVENT_MTU:
        /* Print MTU update info to log */
        ESP_LOGI(TAG, "mtu update event; conn_handle=%d cid=%d mtu=%d",
                 event->mtu.conn_handle, event->mtu.channel_id,
                 event->mtu.value);
        return rc;
    }

    return rc;
}

static void adv_init(void)
{
    /* Local variables */
    int rc = 0;
    char addr_str[18] = {0};

    /* Make sure we have proper BT identity address set (random preferred) */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "device does not have any available bt address!");
        return;
    }

    /* Figure out BT address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
        return;
    }

    /* Printing ADDR */
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to copy device address, error code: %d", rc);
        return;
    }
    format_addr(addr_str, addr_val);
    ESP_LOGI(TAG, "device address: %s", addr_str);

    /* Start advertising. */
    start_advertising();
}

static void on_stack_reset(int reason)
{
    /* On reset, print reset reason to console */
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void)
{
    /* On stack sync, do advertising initialization */
    adv_init();
}

static void nimble_host_config_init(void)
{
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Store host configuration */
    ble_store_config_init();
}

static void nimble_host_task(void *param)
{
    /* Task entry log */
    ESP_LOGI(TAG, "nimble host task has been started!");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}

static void device_info_task(void *param)
{
    ESP_LOGI(TAG, "Device info task has been started!");
    while (1) {
        send_station_id_indication();
        send_station_user_indication();
        send_station_name_indication();
        send_time_zone_indication();
        send_battery_indication();
        send_temp_sensor_indication();

        send_module_indication();
        send_network_indication();
        send_apn_indication();
        send_operator_indication();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void get_temp_sensor_type(void)
{
    #define SHT_ADDRESS 0x44
    i2c_cmd_handle_t cmd;
    xEventGroupWaitBits(event_group, I2C_READY_BIT, pdTRUE, pdTRUE, 500 / portTICK_PERIOD_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT_ADDRESS << 1) | 0, true);
    i2c_master_write_byte(cmd, 0x2400 >> 8, true);
    i2c_master_write_byte(cmd, 0x2400 & 0xFF, true);
    i2c_master_stop(cmd);
    if (i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) == ESP_OK) {
        station.temp_sensor = SHT3x;
        ESP_LOGI(TAG, "Temp sensor type is SHT3x");
    } else {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SHT_ADDRESS << 1) | 0, true);
        i2c_master_write_byte(cmd, 0xFD, true);
        i2c_master_stop(cmd);
        if (i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) == ESP_OK) {
            station.temp_sensor = SHT4x;
            ESP_LOGI(TAG, "Temp sensor type is SHT4x");
        } else {
            station.temp_sensor = 0;
            ESP_LOGE(TAG, "No temp sensor detected");
        }
    }
}

static void set_apn(void)
{
    char apn_set_cmd[64] = "AT+CGDCONT=1,\"IP\",\"";
    strcat(apn_set_cmd, modem.apn);
    strcat(apn_set_cmd, "\"\r");
    sendAT(apn_set_cmd, "OK", 1); // Set APN
}

static void modem_initialize(void)
{
    ESP_LOGI(TAG, "Initializing modem...");

    xEventGroupClearBits(event_group, PROCEED_BIT); // Clear AT response signal
    xEventGroupClearBits(event_group, ERROR_BIT); // Clear AT response signal
    uart_init();
    xTaskCreate(rx_task, "rx_task", RX_BUF_SIZE * 2, NULL, configMAX_PRIORITIES - 1, NULL); // Start AT response receiver task

    modem_power_on();

    while (!sendAT("AT\r", "OK", 1)) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    sendAT("AT+CGMM\r", "AT+CGMM", 1); // Get modem type

    if (modem.module != LBAD0XX1SC) {
        modem.network = CELLULAR;
    } else {
        modem.apn = "data.mono";
    }

    if (modem.network == SATELLITE) {
        if (modem.module == LBAD0XX1SC) {
            AT_response = "BOOTEV";
            xEventGroupWaitBits(event_group, PROCEED_BIT, pdTRUE, pdTRUE, portMAX_DELAY); // Wait for module to be ready
            sendAT("AT+CFUN=0\r", "OK", 5); // Turn off radio
            sendAT("AT%RATACT=\"NBNTN\"\r", "OK", 10); // Enable NB-NTN
            sendAT("AT%IGNSSEV=\"FIX\",1\r", "OK", 1); // Enable GPS fix reporting
            sendAT("AT+CEREG=1\r", "OK", 1); // Enable network registration reporting
            sendAT("AT%IGNSSACT=1\r", "OK", 5); // Start GNSS

            AT_response = "FIX";
            xEventGroupWaitBits(event_group, PROCEED_BIT, pdTRUE, pdTRUE, portMAX_DELAY); // Wait for GPS fix
            sendAT("AT%IGNSSINFO=\"FIX\"\r", "%IGNSSINFO:", 1); // Get GPS info
            //sendAT(set_time_cmd, "OK", 1);
            sendAT("AT%IGNSSACT=0\r", "OK", 5); // Stop GNSS

            sendAT("AT+CGSN\r", "AT+CGSN", 1); // Get IMEI
            sendAT("AT+CFUN=1\r", "CEREG: 5", 0); // Turn on radio
            sendAT("AT+CCLK?\r", "CCLK:", 1); // Get time
            sendAT("AT+CPSMS=1\r", "OK", 1); // Enable PSM
            modem.sleep_mode = PSM;
        }
        //modem_power_off();
    } else if (modem.network == CELLULAR) {
        sendAT("AT+IPR=115200\r", "OK", 1); // Set baud rate
        
        if (modem.module == SIM7070) {
            sendAT("AT+CPSMS=0\r", "OK", 1); // Disable PSM
            sendAT("AT+CMNB=3\r", "OK", 1); // 1 = LTE-M only, 2 = NB-IoT only, 3 = Automatic
            sendAT("AT+CLTS=0\r", "OK", 1); // Disable local time stamp
            //set_apn();
            sendAT("AT+CNMP=38\r", "OK", 1); // 2 = Automatic, 13 = GSM only, 38 = LTE only
            sendAT("AT+CEREG=1\r", "OK", 1); // Enable network registration reporting
        } else if (modem.module == A7670) {
            sendAT("AT+CTZU=0\r", "OK", 1); // Enable local time update
            sendAT("AT+CTZR=0\r", "OK", 1); // Enable time zone reporting
            //set_apn();
            sendAT("AT+CNMP=38\r", "OK", 1); // 2 = Automatic, 13 = GSM only, 38 = LTE only
        } else if (modem.module == LBAD0XX1SC) {
            sendAT("AT+CPSMS=0\r", "OK", 1); // Disable PSM
            sendAT("AT%NOTIFYEV=\"LTIME\",0\r", "OK", 1); // Enable time update
        }
        //sendAT("AT+COPS=1,2,\"310260\",7\r", "OK", 1); //AT&T
        //sendAT("AT+COPS=1,2,\"310410\",7\r", "OK", 1); //AT&T
        //sendAT("AT+COPS=0\r", "OK", 1); //Automatic

        if (modem.module == SIM7070) {
            //AT_response = "PSUTTZ";
            AT_response = "CEREG: 5";
        } else if (modem.module == A7670) {
            AT_response = "CTZV";
        } else if (modem.module == LBAD0XX1SC) {
            AT_response = "LTIME";
        }
        xEventGroupWaitBits(event_group, PROCEED_BIT, pdTRUE, pdTRUE, portMAX_DELAY); // Wait for ready signal
        //modem.time_source = NITZ; // Received NITZ update, update time source
        while (!sendAT("AT+CGREG?\r", "CGREG: 0,5", 1)) { // Wait for network registration
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        sendAT("AT+CGSN\r", "AT+CGSN", 1); // Get IMEI
        sendAT("AT+COPS?\r", "COPS:", 1); // Get operator info
        sendAT("AT+CPSI?\r", "OK", 1); // Get network info
    }
}

void signal_reporting_task(void *param)
{
    while (1) {
        if (modem.module == SIM7070 || modem.module == A7670) {
            sendAT("AT+CSQ\r", "+CSQ:", 1); // Get signal quality
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void station_config(void)
{
    gpio_set_direction(CONFIG_PIN, GPIO_MODE_INPUT); // Configure the button to enable setup mode as an input
    bool user_setup = gpio_get_level(CONFIG_PIN); // Enter setup mode if button is pressed

    esp_err_t ret;

    ret = nvs_flash_init(); // NVS flash initialization
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS flash, error code: %d", ret);
        //return;
    }

    const char *namespace = "settings";
    ESP_ERROR_CHECK(nvs_open(namespace, NVS_READWRITE, &handle));

    size_t size;

    char *key = "station_user";
    size = sizeof(station_user_chr_val);
    if (nvs_get_str(handle, key, NULL, &size) == ESP_OK) { // Check if station user key exists
        char *station_user = malloc(size);
        nvs_get_str(handle, key, station_user, &size);
        ESP_LOGI(TAG, "Station User from NVS: %s", station_user);
        memcpy(station_user_chr_val, station_user, strlen(station_user) + 1);
    } else { // If station user key does not exist, enter setup mode as a new station
        ESP_LOGI(TAG, "New station setup");
        user_setup = true;
    }

    if (user_setup) {
        ESP_LOGI(TAG, "Entering configuration mode...");
        gpio_set_direction(CONFIG_LED_PIN, GPIO_MODE_OUTPUT); 
        gpio_set_level(CONFIG_LED_PIN, 1); // Turn on red LED to indicate setup mode

        ret = nimble_port_init(); // NimBLE stack initialization
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize nimble stack, error code: %d", ret);
            return;
        }

        int rc;

        rc = gap_init(); // GAP service initialization 
        if (rc) {
            ESP_LOGE(TAG, "Failed to initialize GAP service, error code: %d", rc);
            return;
        }

        rc = gatt_svc_init(); // GATT server initialization 
        if (rc) {
            ESP_LOGE(TAG, "Failed to initialize GATT server, error code: %d", rc);
            return;
        }

        nimble_host_config_init(); // NimBLE host configuration initialization

        xTaskCreate(nimble_host_task, "NimBLE Host", 4 * 1024, NULL, 5, NULL);
        xTaskCreate(device_info_task, "Device Info", 4 * 1024, NULL, 5, NULL);

        esp_efuse_mac_get_default(station_id_chr_val); // Load MAC address into station ID characteristic value

        key = "station_name";
        size = sizeof(station_name_chr_val);
        if (nvs_get_str(handle, key, NULL, &size) == ESP_OK) {
            char *station_name = malloc(size);
            nvs_get_str(handle, key, station_name, &size);
            ESP_LOGI(TAG, "Station Name from NVS: %s", station_name);
            memcpy(station_name_chr_val, station_name, strlen(station_name) + 1);
        }
    }    
    
    ESP_LOGI(TAG, "Initializing sensors...");

    ESP_LOGI(TAG, "Checking for temperature/humidity sensor...");
    get_temp_sensor_type();

    ESP_LOGI(TAG, "Checking for rain gauge...");
    gpio_set_direction(RAIN_GAUGE_PIN, GPIO_MODE_INPUT);
    if (gpio_get_level(RAIN_GAUGE_PIN)) {
        station.rain_gauge = true;
        ESP_LOGI(TAG, "Rain gauge connected");
    } else {
        ESP_LOGI(TAG, "Rain gauge not connected");
    }
    //rain_gauge_chr_val[0] = station.rain_gauge;

    ESP_LOGI(TAG, "Checking for anemometer...");
    uint8_t i = 0;
    while (i < 100) { // Take 100 samples of wind speed output
        if (read_ADC(WIND_SPEED_PIN) == ADC_MAX) {
            station.anemometer = true;
            ESP_LOGI(TAG, "Anemometer connnected");
            break;
        }
        i++;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    //anemometer_chr_val[0] = station.anemometer;

    key = "time_zone";
    size = sizeof(time_zone_chr_val);
    //station.time_zone = malloc(size);
    if (nvs_get_str(handle, key, NULL, &size) == ESP_OK) {
        nvs_get_str(handle, key, station.time_zone, &size);
        ESP_LOGI(TAG, "Time zone from NVS: %s", station.time_zone);
    } else {
        ESP_LOGI(TAG, "No time zone in NVS...using default time zone");
        strcpy(station.time_zone, TIME_ZONE_DEFAULT);
        nvs_set_str(handle, key, station.time_zone);
        esp_err_t ret = nvs_commit(handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit data");
        } else {
            ESP_LOGI(TAG, "Time zone saved to NVS");
        }
    }
    memcpy(time_zone_chr_val, station.time_zone, strlen(station.time_zone) + 1);

    key = "battery_type";
    if (nvs_get_u8(handle, key, &station.battery_type) == ESP_OK) {
        ESP_LOGI(TAG, "Battery type from NVS: %d", station.battery_type);
    } else {
        ESP_LOGI(TAG, "No battery type in NVS...using default battery type");
        station.battery_type = BATTERY_TYPE_DEFAULT;
        nvs_set_u8(handle, key, station.battery_type);
        esp_err_t ret = nvs_commit(handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit data");
        } else {
            ESP_LOGI(TAG, "Battery type saved to NVS");
        }
    }

    key = "network_type";
    if (nvs_get_u8(handle, key, &modem.network) == ESP_OK) {
        ESP_LOGI(TAG, "Network type from NVS: %d", modem.network);
    } else {
        ESP_LOGI(TAG, "No network type in NVS...using default network type");
        nvs_set_u8(handle, key, modem.network);
        esp_err_t ret = nvs_commit(handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit data");
        } else {
            ESP_LOGI(TAG, "Network type saved to NVS");
        }
    }

    key = "apn";
    size = sizeof(apn_chr_val);
    modem.apn = malloc(size);
    if (nvs_get_str(handle, key, NULL, &size) == ESP_OK) {
        nvs_get_str(handle, key, modem.apn, &size);
        ESP_LOGI(TAG, "APN from NVS: %s", modem.apn);
    } else {
        ESP_LOGI(TAG, "No APN in NVS...using default APN");
        modem.apn = DEFAULT_APN;
        nvs_set_str(handle, key, modem.apn);
        esp_err_t ret = nvs_commit(handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit data");
        } else {
            ESP_LOGI(TAG, "APN saved to NVS");
        }
    }
    memcpy(apn_chr_val, modem.apn, strlen(modem.apn) + 1);

    modem_initialize();

    if (user_setup) {
        memcpy(operator_chr_val, operator, strlen(operator) + 1);
        TaskHandle_t xHandle = NULL;
        xTaskCreate(signal_reporting_task, "signal_reporting", 2048, NULL, configMAX_PRIORITIES - 1, &xHandle);
        xEventGroupWaitBits(event_group, SETUP_DONE_BIT, pdTRUE, pdTRUE, portMAX_DELAY); // Wait for Bluetooth disconnection to continue
        vTaskDelete(xHandle);
    }

    uint8_t data[1] = {0};
    if (connect_UDP()) {
        send_UDP(data, 1); // Send 1 byte to server to receive time update
        if (modem.module == SIM7070) {
            sendAT("AT+CNACT=0,0\r", "OK", 1);
        }
    }

    #if HARDWARE != ANDGLOBAL
        if (modem.module == SIM7070) {
            //#if HARDWARE == LILYGO
                if (sendAT("AT+CPSMS=1,,,\"11000001\",\"00000000\"\r", "ENTER PSM", 10)){ // Enable PSM with 320 hour period, 0 second active time
                    modem.sleep_mode = PSM;
                    sendAT("AT+CPSMSTATUS=0\r", "OK", 1);
                } else {
                    sendAT("AT+CPSMS=0\r", "OK", 1); // Disable PSM
                }
            //#endif
        } else if (modem.module == A7670) {
            //sendAT("AT+CSCLK=1\r", "OK", 1); // Enable slow clock
        } else if (modem.module == LBAD0XX1SC) {
            //sendAT("AT+CPSMS=1\r", "OK", 1); // Enable PSM
            //modem.sleep_mode = PSM;
        }
    #endif

    if (modem.sleep_mode != PSM) {
        modem.sleep_mode = POWER_OFF;
        modem_power_off();
    }
    
    nvs_close(handle);

    station.measure_interval_s = MEASURE_INTERVAL_S_DEFAULT;
    station.observation_interval_min = OBSERVATION_INTERVAL_MIN_DEFAULT;
    station.transmit_interval_min_default = TRANSMIT_INTERVAL_MIN_DEFAULT;
    station.transmit_interval_min = TRANSMIT_INTERVAL_MIN_DEFAULT;

    if (station.rain_gauge || station.anemometer) {
        ULP_initialize();
    }

    esp_deep_sleep_disable_rom_logging();
}