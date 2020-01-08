#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_err.h>
#include <nvs_flash.h>

#include "config.h"
#include "accessory.h"


static void on_wifi_ready() {
    camera_accessory_init();
}


static esp_err_t event_handler(void *ctx, system_event_t *event) {
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            ESP_LOGI(TAG, "STA start");
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "WiFI ready");

            camera_accessory_set_ip_address((ip4_addr_t){
                .addr = event->event_info.got_ip.ip_info.ip.addr
            });

            on_wifi_ready();
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "STA disconnected");
            esp_wifi_connect();
            break;
        default:
            break;
    }
    return ESP_OK;
}


static void wifi_init() {
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}


void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret != ESP_OK) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK( nvs_flash_init() );
    }

    wifi_init();
}
