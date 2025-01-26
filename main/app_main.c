#include <esp_log.h>
#include <esp_log_buffer.h>
#include <nvs_flash.h>

#include <driver/gpio.h>
#include <driver/i2c.h>

#include <esp_wifi.h>
#include <esp_http_client.h>


// I2C configuration
#define I2C_PORT_NUMBER                         I2C_NUM_0
#define I2C_CLK_FREQUENCY                       10000
#define I2C_SDA_PIN                             GPIO_NUM_4
#define I2C_SCL_PIN                             GPIO_NUM_5
#define I2C_TIMEOUT                             (100 / portTICK_PERIOD_MS)
#define I2C_AHT20_ADDRESS                       0x38

static void i2c_initialize(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_CLK_FREQUENCY,
    };

    i2c_param_config(I2C_PORT_NUMBER, &conf);
    i2c_driver_install(I2C_PORT_NUMBER, conf.mode, 0, 0, 0);
}

static bool i2c_write(uint8_t address, uint8_t *data, size_t length) {
    ESP_LOG_BUFFER_HEX_LEVEL("I2C", data, length, ESP_LOG_DEBUG);

    esp_err_t res = i2c_master_write_to_device(I2C_PORT_NUMBER, address, data, length, I2C_TIMEOUT);
    if (res != ESP_OK) {
        ESP_LOGE("I2C", "error on write, code: 0x%02X", res);
    }

    return res == ESP_OK;
}

static bool i2c_read(uint8_t address, uint8_t *buffer, size_t length) {
    esp_err_t res = i2c_master_read_from_device(I2C_PORT_NUMBER, address, buffer, length, I2C_TIMEOUT);
    if (res != ESP_OK) {
        ESP_LOGI("I2C", "error on read, code: 0x%02X", res);
    } else {
        ESP_LOG_BUFFER_HEX_LEVEL("I2C", buffer, length, ESP_LOG_DEBUG);
    }

    return res == ESP_OK;
}

// AHT20 temperature & humidity sensor
//
// AHT20 ESP IoT Solution
// driver: https://github.com/espressif/esp-iot-solution/tree/master/components/sensors/humiture/aht20
//
// AHT20 sensor
// website: http://www.aosong.com/en/products-32.html
// documentation: https://files.seeedstudio.com/wiki/Grove-AHT20_I2C_Industrial_Grade_Temperature_and_Humidity_Sensor/AHT20-datasheet-2020-4-16.pdf
// I2C example: esp-idf-v4.4.2/examples/peripherals/i2c/i2c_simple/main/i2c_simple_main.c
// 3rdParty library: https://github.com/adafruit/Adafruit_AHTX0

static void aht20_initialize() {
    uint8_t request[] = {0xBE};
    i2c_write(I2C_AHT20_ADDRESS, request, sizeof(request));
}

static bool aht20_trigger_measurement() {
    // TODO
    return false;
}

static bool aht20_read_data(uint8_t *buffer, size_t length) {
    // TODO
    return false;
}

static float aht20_get_humidity(uint8_t *buffer) {
    // TODO
    return 0;
}


static float aht20_get_temperature(uint8_t *buffer) {
    // TODO
    return 0;
}


static void wait_ms(unsigned delay) {
    vTaskDelay(delay / portTICK_PERIOD_MS);
}


static void wifi_on_event(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ESP_LOGI("WIFI", "event %d", (int)event_id);

    if (event_base != WIFI_EVENT)
        return;

    switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            break;

        default:
            break;
    }
}

static void ip_on_event(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base != IP_EVENT)
        return;

    if (event_id != IP_EVENT_STA_GOT_IP)
        return;

    ip_event_got_ip_t *event = (ip_event_got_ip_t*) event_data;
    ESP_LOGI("WIFI", "AP address:" IPSTR, IP2STR(&event->ip_info.ip));
}



static void wifi_init() {
    // see https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-guides/wifi.html
    // see https://github.com/espressif/esp-idf/tree/v5.4/examples/wifi/getting_started/station
    esp_netif_init();

    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t wifi_handler_instance;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_on_event, NULL, &wifi_handler_instance);

    esp_event_handler_instance_t ip_handler_instance;
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &ip_on_event, NULL, &ip_handler_instance);

    wifi_config_t wifi_config = {
        .sta = {
            // .ssid = "",
            // .password = "",
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

static void cloud_send_temperature(void) {
    // see https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-reference/protocols/esp_http_client.html
    esp_http_client_config_t config = {
        //
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_header(client, ...);

    // const char *data = ""
    esp_http_client_set_post_field(client, ...);

    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

void app_main(void) {
    // Initialize Non-Volatile Memory
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI("NVS", "Initializing NVS...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    // Initialize I2C interface
    i2c_initialize();

    // Initialize WiFi stack
    wifi_init();

    while (1) {
        wait_ms(1000);
    };
}
