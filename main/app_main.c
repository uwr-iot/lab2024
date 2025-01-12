#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_nimble_hci.h>
#include <driver/gpio.h>
#include <driver/i2c.h>

#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <host/ble_hs.h>
#include <host/util/util.h>

#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>


// ESP-IDF
// BLE documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/bluetooth/nimble/index.html
// BLE Heart Rate Measurement Example: https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/nimble/blehr

// Heart rate Sensor Configuration

// BLE specification: Assigned Numbers
// link: https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Assigned_Numbers/out/en/Assigned_Numbers.pdf?v=1736689131263
//
// This is a regularly updated document listing assigned numbers, codes, and
// identifiers in the Bluetooth specifications.

// BLE specification: GATT Specification Supplement 5
// link: https://www.bluetooth.org/DocMan/handlers/DownloadDoc.ashx?doc_id=524815
//
// This specification contains the normative definitions for all GATT characteristics and
// characteristic descriptors, with the exception of those defined in the Bluetooth Core Specification
// or in Bluetooth Service specifications.

#define GATT_HRS_UUID                           0x180D
#define GATT_HRS_MEASUREMENT_UUID               0x2A37
#define GATT_HRS_BODY_SENSOR_LOC_UUID           0x2A38

// Variable to simulate heart beats
static uint8_t heartrate = 90;


static void wait_ms(unsigned delay) {
    vTaskDelay(delay / portTICK_PERIOD_MS);
}


static int read_heart_rate_measurement(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
   uint8_t hrm[2];
    hrm[0] = 0b00000000;    // Field flags
    hrm[1] = heartrate;     // Heart Rate Measurement Value

    // More about Mbufs -> https://mynewt.apache.org/latest/os/core_os/mbuf/mbuf.html
    os_mbuf_append(ctxt->om, &hrm, sizeof(hrm));

    return 0;
}


static int read_body_sensor_location(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return BLE_ATT_ERR_INSUFFICIENT_RES;
}


static const struct ble_gatt_svc_def kBleServices[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATT_HRS_UUID),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                .uuid = BLE_UUID16_DECLARE(GATT_HRS_MEASUREMENT_UUID),
                .access_cb = read_heart_rate_measurement,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                .uuid = BLE_UUID16_DECLARE(GATT_HRS_BODY_SENSOR_LOC_UUID),
                .access_cb = read_body_sensor_location,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                0,  // no more characteristics
            },
        }
    }, {
        0,  // no more services
    },
};


static void start_advertisement(void);


static int on_ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI("BLE GAP Event", "Connected");
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI("BLE GAP Event", "Disconnected");
            start_advertisement();
            break;

        default:
            ESP_LOGI("BLE GAP Event", "Type: 0x%02X", event->type);
            break;
    }

    return 0;
}


static void start_advertisement(void) {
    struct ble_gap_adv_params adv_parameters;
    memset(&adv_parameters, 0, sizeof(adv_parameters));

    adv_parameters.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;

    if (ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                         &adv_parameters,
                         on_ble_gap_event, NULL) != 0) {
        ESP_LOGE("BLE", "Can't start Advertisement");
        return;
    }

    ESP_LOGI("BLE", "Advertisement started...");
}


static void set_device_name(const char *device_name) {
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    fields.name = (uint8_t*) device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    if (ble_gap_adv_set_fields(&fields) != 0) {
        ESP_LOGE("BLE", "Can't configure BLE advertisement fields");
        return;
    }

    ble_svc_gap_device_name_set(device_name);
}


static void start_ble_service(void *param) {
    ESP_LOGI("BLE task", "BLE Host Task Started");

    nimble_port_run();
    nimble_port_freertos_deinit();
}


void app_main(void) {
    // Initialize Non-Volatile Memory
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI("NVS", "Initializing NVS...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    // Initialize BLE peripheral
    nimble_port_init();
    esp_nimble_hci_init();

    // Initialize BLE library (nimble)
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Configure BLE library (nimble)
    int rc = ble_gatts_count_cfg(kBleServices);
    if (rc != 0) {
        ESP_LOGE("BLE GATT", "Service registration failed");
        goto error;
    }

    // Register all services
    rc = ble_gatts_add_svcs(kBleServices);
    if (rc != 0) {
        ESP_LOGE("BLE GATT", "Service registration failed");
        goto error;
    }

    // Start BLE stack
    nimble_port_freertos_init(start_ble_service);

    // Start BLE device
    set_device_name("IoT LAB device");
    start_advertisement();

error:
    while (1) {
        wait_ms(1000);
    };
}
