#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "hid_ble_mouse";
static const char local_device_name[] = "ESP32HIDMouse";

// ---- UUIDs ----
static constexpr uint16_t UUID_HID_SERVICE        = 0x1812;
static constexpr uint16_t UUID_HID_INFO           = 0x2A4A;
static constexpr uint16_t UUID_HID_REPORT_MAP     = 0x2A4B;
static constexpr uint16_t UUID_HID_CONTROL_POINT  = 0x2A4C;
static constexpr uint16_t UUID_HID_REPORT         = 0x2A4D;
static constexpr uint16_t UUID_HID_PROTOCOL_MODE  = 0x2A4E;

static constexpr uint16_t UUID_PRIMARY_SERVICE    = ESP_GATT_UUID_PRI_SERVICE;   // 0x2800
static constexpr uint16_t UUID_CHAR_DECL          = ESP_GATT_UUID_CHAR_DECLARE;  // 0x2803
static constexpr uint16_t UUID_CCCD               = ESP_GATT_UUID_CHAR_CLIENT_CONFIG; // 0x2902
static constexpr uint16_t UUID_REPORT_REF_DESC    = ESP_GATT_UUID_RPT_REF_DESCR; // 0x2908

// ---- HID Report Map (mouse, 3 botones + X/Y + wheel) ----
static const uint8_t hid_report_map[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x02,       // Usage (Mouse)
    0xA1, 0x01,       // Collection (Application)
    0x09, 0x01,       //   Usage (Pointer)
    0xA1, 0x00,       //   Collection (Physical)
    0x05, 0x09,       //     Usage Page (Buttons)
    0x19, 0x01,       //     Usage Minimum (1)
    0x29, 0x03,       //     Usage Maximum (3)
    0x15, 0x00,       //     Logical Minimum (0)
    0x25, 0x01,       //     Logical Maximum (1)
    0x95, 0x03,       //     Report Count (3)
    0x75, 0x01,       //     Report Size (1)
    0x81, 0x02,       //     Input (Data,Var,Abs)
    0x95, 0x01,       //     Report Count (1)
    0x75, 0x05,       //     Report Size (5)
    0x81, 0x03,       //     Input (Const,Var,Abs)
    0x05, 0x01,       //     Usage Page (Generic Desktop)
    0x09, 0x30,       //     Usage (X)
    0x09, 0x31,       //     Usage (Y)
    0x09, 0x38,       //     Usage (Wheel)
    0x15, 0x81,       //     Logical Minimum (-127)
    0x25, 0x7F,       //     Logical Maximum (127)
    0x75, 0x08,       //     Report Size (8)
    0x95, 0x03,       //     Report Count (3)
    0x81, 0x06,       //     Input (Data,Var,Rel)
    0xC0,             //   End Collection
    0xC0              // End Collection
};

// HID Information: bcdHID=0x0111, country=0, flags=0x02
static uint8_t hid_info_val[4] = {0x11, 0x01, 0x00, 0x02};
static uint8_t protocol_mode_val = 0x01; // Report Protocol
static uint8_t control_point_val = 0x00;

// Input Report (botones, x, y, wheel)
static uint8_t input_report_val[4] = {0x00, 0x00, 0x00, 0x00};
// Report Reference: report_id=1, report_type=1(Input)
static uint8_t report_ref_input[2] = {0x01, 0x01};
static uint8_t cccd_val[2] = {0x00, 0x00};

enum hid_idx_t {
    HID_IDX_SVC = 0,

    HID_IDX_PROTOCOL_MODE_CHAR,
    HID_IDX_PROTOCOL_MODE_VAL,

    HID_IDX_HID_INFO_CHAR,
    HID_IDX_HID_INFO_VAL,

    HID_IDX_REPORT_MAP_CHAR,
    HID_IDX_REPORT_MAP_VAL,

    HID_IDX_CONTROL_POINT_CHAR,
    HID_IDX_CONTROL_POINT_VAL,

    HID_IDX_INPUT_REPORT_CHAR,
    HID_IDX_INPUT_REPORT_VAL,
    HID_IDX_INPUT_REPORT_CCCD,
    HID_IDX_INPUT_REPORT_REF,

    HID_IDX_NB
};

static uint16_t hid_handle_table[HID_IDX_NB] = {0};
static esp_gatt_if_t s_gatts_if = ESP_GATT_IF_NONE;
static uint16_t s_conn_id = 0xFFFF;
static bool s_notify_enabled = false;
static bool s_connected = false;

static uint8_t hid_char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static uint8_t hid_char_prop_read_write_nr = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static uint8_t hid_char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

// Advertising (legacy)
static uint8_t hid_service_uuid16[2] = {0x12, 0x18};
static bool adv_config_done = false;
static bool scan_rsp_config_done = false;
static esp_ble_adv_data_t adv_data = {};
static esp_ble_adv_data_t scan_rsp_data = {};
static esp_ble_adv_params_t adv_params = {};

static const esp_gatts_attr_db_t hid_gatt_db[HID_IDX_NB] = {
    // Service
    [HID_IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_PRIMARY_SERVICE, ESP_GATT_PERM_READ,
         sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&UUID_HID_SERVICE}
    },

    // Protocol Mode
    [HID_IDX_PROTOCOL_MODE_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECL, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), &hid_char_prop_read_write_nr}
    },
    [HID_IDX_PROTOCOL_MODE_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_HID_PROTOCOL_MODE, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint8_t), sizeof(protocol_mode_val), &protocol_mode_val}
    },

    // HID Information
    [HID_IDX_HID_INFO_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECL, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), &hid_char_prop_read}
    },
    [HID_IDX_HID_INFO_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_HID_INFO, ESP_GATT_PERM_READ,
         sizeof(hid_info_val), sizeof(hid_info_val), hid_info_val}
    },

    // Report Map
    [HID_IDX_REPORT_MAP_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECL, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), &hid_char_prop_read}
    },
    [HID_IDX_REPORT_MAP_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_HID_REPORT_MAP, ESP_GATT_PERM_READ,
         sizeof(hid_report_map), sizeof(hid_report_map), (uint8_t *)hid_report_map}
    },

    // HID Control Point
    [HID_IDX_CONTROL_POINT_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECL, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), &hid_char_prop_read_write_nr}
    },
    [HID_IDX_CONTROL_POINT_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_HID_CONTROL_POINT, ESP_GATT_PERM_WRITE,
         sizeof(uint8_t), sizeof(control_point_val), &control_point_val}
    },

    // Input Report
    [HID_IDX_INPUT_REPORT_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECL, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), &hid_char_prop_read_notify}
    },
    [HID_IDX_INPUT_REPORT_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_HID_REPORT, ESP_GATT_PERM_READ,
         sizeof(input_report_val), sizeof(input_report_val), input_report_val}
    },
    [HID_IDX_INPUT_REPORT_CCCD] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CCCD, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), sizeof(cccd_val), cccd_val}
    },
    [HID_IDX_INPUT_REPORT_REF] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_REPORT_REF_DESC, ESP_GATT_PERM_READ,
         sizeof(report_ref_input), sizeof(report_ref_input), report_ref_input}
    },
};

static void start_advertising_if_ready()
{
    if (adv_config_done && scan_rsp_config_done) {
        esp_ble_gap_start_advertising(&adv_params);
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done = true;
        start_advertising_if_ready();
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        scan_rsp_config_done = true;
        start_advertising_if_ready();
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "ADV start status=%d", param->adv_start_cmpl.status);
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        s_gatts_if = gatts_if;
        ESP_ERROR_CHECK(esp_ble_gap_set_device_name(local_device_name));
        ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
        ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&scan_rsp_data));
        ESP_ERROR_CHECK(esp_ble_gatts_create_attr_tab(hid_gatt_db, gatts_if, HID_IDX_NB, 0));
        break;
    }

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK && param->add_attr_tab.num_handle == HID_IDX_NB) {
            memcpy(hid_handle_table, param->add_attr_tab.handles, sizeof(hid_handle_table));
            ESP_ERROR_CHECK(esp_ble_gatts_start_service(hid_handle_table[HID_IDX_SVC]));
            ESP_LOGI(TAG, "HID attribute table creada");
        } else {
            ESP_LOGE(TAG, "Error creando tabla HID, status=%d", param->add_attr_tab.status);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        s_connected = true;
        s_conn_id = param->connect.conn_id;
        ESP_LOGI(TAG, "Conectado");
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        s_connected = false;
        s_notify_enabled = false;
        s_conn_id = 0xFFFF;
        esp_ble_gap_start_advertising(&adv_params);
        ESP_LOGI(TAG, "Desconectado, advertising restart");
        break;

    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == hid_handle_table[HID_IDX_INPUT_REPORT_CCCD] && param->write.len == 2) {
            uint16_t cccd = param->write.value[0] | (param->write.value[1] << 8);
            s_notify_enabled = (cccd & 0x0001);
            ESP_LOGI(TAG, "CCCD notify=%s", s_notify_enabled ? "ON" : "OFF");
        }
        break;

    default:
        break;
    }
}

static void send_mouse_report(uint8_t buttons, int8_t x, int8_t y, int8_t wheel){
    if (!s_connected || !s_notify_enabled || s_gatts_if == ESP_GATT_IF_NONE) {
        return;
    }

    input_report_val[0] = buttons;
    input_report_val[1] = static_cast<uint8_t>(x);
    input_report_val[2] = static_cast<uint8_t>(y);
    input_report_val[3] = static_cast<uint8_t>(wheel);

    esp_ble_gatts_set_attr_value(
        hid_handle_table[HID_IDX_INPUT_REPORT_VAL],
        sizeof(input_report_val),
        input_report_val);

    esp_ble_gatts_send_indicate(
        s_gatts_if,
        s_conn_id,
        hid_handle_table[HID_IDX_INPUT_REPORT_VAL],
        sizeof(input_report_val),
        input_report_val,
        false // notify (sin confirmación)
    );
}
static void mouse_demo_task(void *arg){
    (void)arg;
    while (true) {
        // mueve +10 en X y vuelve -10
        send_mouse_report(0x00, 10, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
        send_mouse_report(0x00, static_cast<int8_t>(-10), 0, 0);
        vTaskDelay(pdMS_TO_TICKS(1800));
    }
}

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bluedroid_cfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0x55));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(200));

    memset(&adv_data, 0, sizeof(adv_data));
    adv_data.set_scan_rsp = false;
    adv_data.include_name = true;
    adv_data.appearance = 0x03C2; // Mouse
    adv_data.service_uuid_len = sizeof(hid_service_uuid16);
    adv_data.p_service_uuid = hid_service_uuid16;
    adv_data.flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;

    memset(&scan_rsp_data, 0, sizeof(scan_rsp_data));
    scan_rsp_data.set_scan_rsp = true;
    scan_rsp_data.include_name = true;
    scan_rsp_data.appearance = 0x03C2;
    scan_rsp_data.service_uuid_len = sizeof(hid_service_uuid16);
    scan_rsp_data.p_service_uuid = hid_service_uuid16;

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.adv_int_min = 0x20;
    adv_params.adv_int_max = 0x40;
    adv_params.adv_type = ADV_TYPE_IND;
    adv_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    adv_params.channel_map = ADV_CHNL_ALL;
    adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

    ESP_LOGI(TAG, "Patch 2 listo: HID attr table (ReportMap + InputReport + CCCD)");

    xTaskCreate(mouse_demo_task, "mouse_demo_task", 4096, nullptr, 5, nullptr);
}