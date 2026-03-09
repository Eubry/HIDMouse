/**
 * @file main.cpp
 * @brief ESP32-S3 BLE HID Mouse using MPU-6050
 *
 * Pins:
 *   SDA  -> GPIO 8
 *   SCL  -> GPIO 9
 *   INT  -> GPIO 10
 *
 * Gyro axes mapping (adjust if cursor moves in wrong direction):
 *   Gyro X (pitch) -> mouse Y
 *   Gyro Y (roll)  -> mouse X
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_hidd.h"
#include "esp_hid_gap.h"

// ============================================================
//  Pin definitions
// ============================================================
#define MPU6050_SDA_PIN     GPIO_NUM_8
#define MPU6050_SCL_PIN     GPIO_NUM_9
#define MPU6050_INT_PIN     GPIO_NUM_10

// ============================================================
//  MPU-6050 register map
// ============================================================
#define MPU6050_ADDR            0x68    // AD0 = GND
#define MPU6050_REG_SMPLRT_DIV  0x19
#define MPU6050_REG_CONFIG      0x1A
#define MPU6050_REG_GYRO_CFG    0x1B
#define MPU6050_REG_INT_PIN_CFG 0x37
#define MPU6050_REG_INT_ENABLE  0x38
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_PWR_MGMT_1  0x6B

// ============================================================
//  Tuning constants
// ============================================================
/** Divide raw gyro value (±32767 at ±250 °/s) down to mouse delta.
 *  Lower value = more sensitive.  Increase to slow the cursor. */
#define GYRO_SENSITIVITY    512

/** Dead-zone in raw gyro LSB (131 LSB = 1 °/s).
 *  Values inside ±DEAD_ZONE are treated as zero to suppress drift. */
#define GYRO_DEAD_ZONE      300

static const char *TAG = "BLE_HID_MOUSE";

// ============================================================
//  I2C handles
// ============================================================
static i2c_master_bus_handle_t  s_i2c_bus;
static i2c_master_dev_handle_t  s_mpu_dev;

// ============================================================
//  BLE HID state
// ============================================================
typedef struct {
    TaskHandle_t     task_hdl;
    esp_hidd_dev_t  *hid_dev;
} ble_hid_param_t;

static ble_hid_param_t s_ble_hid_param = {
    .task_hdl = NULL,
    .hid_dev = NULL,
};

// Signalled from the INT GPIO ISR; unblocks the mouse task.
static SemaphoreHandle_t s_data_ready_sem = NULL;

// ============================================================
//  HID report descriptor – standard 4-byte relative mouse
//  (buttons 1 byte + padding 5 bits | X 1 byte | Y 1 byte | wheel 1 byte)
// ============================================================
static const uint8_t s_mouse_report_map[] = {
    0x05, 0x01,  // Usage Page (Generic Desktop)
    0x09, 0x02,  // Usage (Mouse)
    0xA1, 0x01,  // Collection (Application)
    0x09, 0x01,  //   Usage (Pointer)
    0xA1, 0x00,  //   Collection (Physical)
    // Buttons 1-3
    0x05, 0x09,  //     Usage Page (Button)
    0x19, 0x01,  //     Usage Minimum (1)
    0x29, 0x03,  //     Usage Maximum (3)
    0x15, 0x00,  //     Logical Minimum (0)
    0x25, 0x01,  //     Logical Maximum (1)
    0x95, 0x03,  //     Report Count (3)
    0x75, 0x01,  //     Report Size (1)
    0x81, 0x02,  //     Input (Data, Variable, Absolute)
    // Padding (5 bits)
    0x95, 0x01,  //     Report Count (1)
    0x75, 0x05,  //     Report Size (5)
    0x81, 0x03,  //     Input (Constant)
    // X, Y, Wheel – relative signed bytes
    0x05, 0x01,  //     Usage Page (Generic Desktop)
    0x09, 0x30,  //     Usage (X)
    0x09, 0x31,  //     Usage (Y)
    0x09, 0x38,  //     Usage (Wheel)
    0x15, 0x81,  //     Logical Minimum (-127)
    0x25, 0x7F,  //     Logical Maximum (127)
    0x75, 0x08,  //     Report Size (8)
    0x95, 0x03,  //     Report Count (3)
    0x81, 0x06,  //     Input (Data, Variable, Relative)
    0xC0,        //   End Collection
    0xC0,        // End Collection
};

static esp_hid_raw_report_map_t s_report_maps[] = {
    { .data = s_mouse_report_map, .len = sizeof(s_mouse_report_map) },
};

static esp_hid_device_config_t s_hid_config = {
    .vendor_id         = 0x16C0,
    .product_id        = 0x05DF,
    .version           = 0x0100,
    .device_name       = "ESP32-S3 Mouse",
    .manufacturer_name = "Espressif",
    .serial_number     = "0000000001",
    .report_maps       = s_report_maps,
    .report_maps_len   = 1,
};

// ============================================================
//  MPU-6050 helpers
// ============================================================
static esp_err_t mpu_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(s_mpu_dev, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_mpu_dev, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t mpu6050_init(void)
{
    esp_err_t ret;

    // Wake up (clear SLEEP bit)
    ret = mpu_write(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    // DLPF 44 Hz bandwidth → gyro sample rate = 1 kHz
    ret = mpu_write(MPU6050_REG_CONFIG, 0x03);
    if (ret != ESP_OK) return ret;

    // Sample rate = 1000 / (1 + 9) = 100 Hz
    ret = mpu_write(MPU6050_REG_SMPLRT_DIV, 9);
    if (ret != ESP_OK) return ret;

    // Gyro full-scale ±250 °/s (131 LSB/°/s)
    ret = mpu_write(MPU6050_REG_GYRO_CFG, 0x00);
    if (ret != ESP_OK) return ret;

    // INT pin: active-high, push-pull, latched, cleared by any read
    ret = mpu_write(MPU6050_REG_INT_PIN_CFG, 0x30);
    if (ret != ESP_OK) return ret;

    // Enable DATA_RDY interrupt
    ret = mpu_write(MPU6050_REG_INT_ENABLE, 0x01);
    return ret;
}

static esp_err_t mpu6050_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t raw[6];
    esp_err_t ret = mpu_read(MPU6050_REG_GYRO_XOUT_H, raw, sizeof(raw));
    if (ret != ESP_OK) return ret;
    *gx = (int16_t)((raw[0] << 8) | raw[1]);
    *gy = (int16_t)((raw[2] << 8) | raw[3]);
    *gz = (int16_t)((raw[4] << 8) | raw[5]);
    return ESP_OK;
}

// ============================================================
//  GPIO interrupt (MPU-6050 DATA_RDY → INT pin)
// ============================================================
static void IRAM_ATTR mpu6050_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_data_ready_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

static esp_err_t mpu6050_int_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << MPU6050_INT_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,   // keep pin low when idle
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) return ret;

    // ESP_ERR_INVALID_STATE means ISR service already installed – that's fine.
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    return gpio_isr_handler_add(MPU6050_INT_PIN, mpu6050_isr, NULL);
}

// ============================================================
//  Mouse helpers
// ============================================================
static inline int8_t clamp8(int32_t v)
{
    if (v >  127) return  127;
    if (v < -127) return -127;
    return (int8_t)v;
}

static void send_mouse(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel)
{
    uint8_t buf[4] = {buttons, (uint8_t)dx, (uint8_t)dy, (uint8_t)wheel};
    esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, 0, 0, buf, sizeof(buf));
}

// ============================================================
//  Mouse task – runs while BLE HID is connected
// ============================================================
static void mouse_task(void *pvParameters)
{
    int16_t gx, gy, gz;

    while (1) {
        // Wait for DATA_RDY interrupt (20 ms timeout = polling fallback at 50 Hz)
        xSemaphoreTake(s_data_ready_sem, pdMS_TO_TICKS(20));

        if (mpu6050_read_gyro(&gx, &gy, &gz) != ESP_OK) {
            ESP_LOGW(TAG, "MPU-6050 read error");
            continue;
        }

        // Apply dead zone in raw LSB
        if (gx > -GYRO_DEAD_ZONE && gx < GYRO_DEAD_ZONE) gx = 0;
        if (gy > -GYRO_DEAD_ZONE && gy < GYRO_DEAD_ZONE) gy = 0;

        // Scale and clamp
        // Gyro X (pitch) → cursor Y,  Gyro Y (roll) → cursor X
        // Negate dy so that tilting forward moves cursor up.
        int8_t dx = clamp8( gy / GYRO_SENSITIVITY);
        int8_t dy = clamp8(-gx / GYRO_SENSITIVITY);

        if (dx != 0 || dy != 0) {
            send_mouse(0, dx, dy, 0);
        }
    }
}

// ============================================================
//  BLE HID task lifecycle (called by esp_hid_gap.c extern refs)
// ============================================================
extern "C" void ble_hid_task_start_up(void)
{
    if (s_ble_hid_param.task_hdl) return;
    xTaskCreate(mouse_task, "mouse_task", 4096, NULL,
                configMAX_PRIORITIES - 3, &s_ble_hid_param.task_hdl);
}

extern "C" void ble_hid_task_shut_down(void)
{
    if (s_ble_hid_param.task_hdl) {
        vTaskDelete(s_ble_hid_param.task_hdl);
        s_ble_hid_param.task_hdl = NULL;
    }
}

// ============================================================
//  BLE HIDD event callback
// ============================================================
static void ble_hidd_event_cb(void *handler_args, esp_event_base_t base,
                               int32_t id, void *event_data)
{
    esp_hidd_event_t         event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t   *param = (esp_hidd_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDD_START_EVENT:
        ESP_LOGI(TAG, "HID START – starting advertisement");
        esp_hid_ble_gap_adv_start();
        break;

    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG, "HID CONNECT");
        break;

    case ESP_HIDD_PROTOCOL_MODE_EVENT:
        ESP_LOGI(TAG, "PROTOCOL MODE: %s",
                 param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;

    case ESP_HIDD_CONTROL_EVENT:
        if (param->control.control) {
            ble_hid_task_start_up();   // EXIT_SUSPEND
        } else {
            ble_hid_task_shut_down();  // SUSPEND
        }
        break;

    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG, "HID DISCONNECT: %s",
                 esp_hid_disconnect_reason_str(
                     esp_hidd_dev_transport_get(param->disconnect.dev),
                     param->disconnect.reason));
        ble_hid_task_shut_down();
        esp_hid_ble_gap_adv_start();   // re-advertise
        break;

    case ESP_HIDD_STOP_EVENT:
        ESP_LOGI(TAG, "HID STOP");
        break;

    default:
        break;
    }
}

// ============================================================
//  I2C bus initialisation
// ============================================================
static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port              = I2C_NUM_0,
        .sda_io_num            = MPU6050_SDA_PIN,
        .scl_io_num            = MPU6050_SCL_PIN,
        .clk_source            = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt     = 7,
        .intr_priority         = 0,
        .trans_queue_depth     = 0,
        .flags = { .enable_internal_pullup = 1, .allow_pd = 0 },
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (ret != ESP_OK) return ret;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_ADDR,
        .scl_speed_hz    = 400000,
        .scl_wait_us     = 0,
        .flags           = { .disable_ack_check = 0 },
    };
    return i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_mpu_dev);
}

// ============================================================
//  app_main
// ============================================================
extern "C" void app_main(void)
{
    esp_err_t ret;

    // ---- NVS (required by BT stack) ----
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // ---- I2C + MPU-6050 ----
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialised (SDA=%d, SCL=%d)", MPU6050_SDA_PIN, MPU6050_SCL_PIN);

    s_data_ready_sem = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(s_data_ready_sem ? ESP_OK : ESP_ERR_NO_MEM);

    ESP_ERROR_CHECK(mpu6050_init());
    ESP_LOGI(TAG, "MPU-6050 initialised");

    ESP_ERROR_CHECK(mpu6050_int_init());
    ESP_LOGI(TAG, "MPU-6050 INT on GPIO %d", MPU6050_INT_PIN);

    // ---- BLE HID ----
    ESP_LOGI(TAG, "Initialising BLE HID");
    ESP_ERROR_CHECK(esp_hid_gap_init(HIDD_BLE_MODE));
    ESP_ERROR_CHECK(esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_MOUSE,
                                              s_hid_config.device_name));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler));
    ESP_ERROR_CHECK(esp_hidd_dev_init(&s_hid_config, ESP_HID_TRANSPORT_BLE,
                                      ble_hidd_event_cb,
                                      &s_ble_hid_param.hid_dev));

    ESP_LOGI(TAG, "Advertising as \"%s\" – pair with passkey 1234",
             s_hid_config.device_name);
}
