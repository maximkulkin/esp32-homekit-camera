#include <stdio.h>
#include <string.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_err.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include <camera.h>

#define STREAMING_STATUS_AVAILABLE 0
#define STREAMING_STATUS_IN_USE 1
#define STREAMING_STATUS_UNAVAILABLE 2


#define TAG "esp32_camera"


#define CAMERA_PIXEL_FORMAT CAMERA_PF_GRAYSCALE
#define CAMERA_FRAME_SIZE CAMERA_FS_VGA
static camera_pixelformat_t s_pixel_format;


void on_wifi_ready();

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            printf("STA start\n");
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            printf("WiFI ready\n");
            on_wifi_ready();
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            printf("STA disconnected\n");
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

const int camera_led_gpio = CONFIG_PIN_LED;
bool camera_led_on = false;

void camera_led_set(bool on) {
    gpio_set_level(camera_led_gpio, on ? 1 : 0);
    camera_led_on = on;
}

void camera_identify_task(void *_args) {
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            camera_led_set(true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            camera_led_set(false);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    camera_led_set(true);

    vTaskDelete(NULL);
}

void camera_identify(homekit_value_t _value) {
    printf("Camera identify\n");
    xTaskCreate(camera_identify_task, "Camera identify", 512, NULL, 2, NULL);
}

homekit_value_t camera_streaming_status_get() {
    tlv_values_t *tlv = tlv_new();
    tlv_add_integer_value(tlv, 1, STREAMING_STATUS_AVAILABLE);
    return HOMEKIT_TLV(tlv);
}

homekit_value_t camera_setup_endpoints_get() {
    return HOMEKIT_TLV(tlv_new());
}

void camera_setup_endpoints_set(homekit_value_t value) {
    if (value.format != homekit_format_bool) {
        printf("Invalid value format: %d\n", value.format);
        return;
    }

    // TODO:
}

homekit_value_t camera_selected_rtp_configuration_get() {
    return HOMEKIT_TLV(tlv_new());
}

void camera_selected_rtp_configuration_set(homekit_value_t value) {
    if (value.format != homekit_format_bool) {
        printf("Invalid value format: %d\n", value.format);
        return;
    }

    // TODO:
}

tlv_values_t supported_video_config = {};
tlv_values_t supported_audio_config = {};
tlv_values_t supported_rtp_config = {};


void camera_accessory_init() {
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    gpio_set_direction(camera_led_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(camera_led_gpio, 1);

    camera_config_t camera_config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pin_d0 = CONFIG_PIN_D0,
        .pin_d1 = CONFIG_PIN_D1,
        .pin_d2 = CONFIG_PIN_D2,
        .pin_d3 = CONFIG_PIN_D3,
        .pin_d4 = CONFIG_PIN_D4,
        .pin_d5 = CONFIG_PIN_D5,
        .pin_d6 = CONFIG_PIN_D6,
        .pin_d7 = CONFIG_PIN_D7,
        .pin_xclk = CONFIG_PIN_XCLK,
        .pin_pclk = CONFIG_PIN_PCLK,
        .pin_vsync = CONFIG_PIN_VSYNC,
        .pin_href = CONFIG_PIN_HREF,
        .pin_sscb_sda = CONFIG_PIN_SDA,
        .pin_sscb_scl = CONFIG_PIN_SCL,
        .pin_reset = CONFIG_PIN_RESET,
        .xclk_freq_hz = CONFIG_XCLK_FREQ,
    };

    camera_model_t camera_model;
    esp_err_t err = camera_probe(&camera_config, &camera_model);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera probe failed with error 0x%x", err);
        return;
    }

    if (camera_model == CAMERA_OV7725) {
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        camera_config.frame_size = CAMERA_FRAME_SIZE;
        ESP_LOGI(TAG, "Detected OV7725 camera, using %s bitmap format",
                 CAMERA_PIXEL_FORMAT == CAMERA_PF_GRAYSCALE ?  "grayscale" : "RGB565");
    } else if (camera_model == CAMERA_OV2640) {
        ESP_LOGI(TAG, "Detected OV2640 camera, using JPEG format");
        s_pixel_format = CAMERA_PF_JPEG;
        camera_config.frame_size = CAMERA_FRAME_SIZE;
        camera_config.jpeg_quality = 15;
    } else {
        ESP_LOGE(TAG, "Camera not supported");
        return;
    }

    camera_config.pixel_format = s_pixel_format;
    err = camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    tlv_values_t *video_codec_params = tlv_new();
    tlv_add_integer_value(video_codec_params, 1, 1);  // Profile ID
    tlv_add_integer_value(video_codec_params, 2, 0);  // Level
    tlv_add_integer_value(video_codec_params, 3, 0);  // Packetization mode
    tlv_add_integer_value(video_codec_params, 4, 0);  // CVO Enabled
    tlv_add_integer_value(video_codec_params, 5, 1);  // CVO ID

    tlv_values_t *video_attributes = tlv_new();
    tlv_add_integer_value(video_attributes, 1, 640);  // Image width
    tlv_add_integer_value(video_attributes, 2, 480);  // Image height
    tlv_add_integer_value(video_attributes, 3, 30);  // Frame rate

    tlv_values_t *video_codec_config = tlv_new();
    tlv_add_integer_value(video_codec_config, 1, 0);  // Video codec type
    tlv_add_tlv_value(video_codec_config, 2, video_codec_params);  // Video codec params
    tlv_add_tlv_value(video_codec_config, 3, video_attributes);  // Video attributes

    tlv_add_tlv_value(&supported_video_config, 1, video_codec_config);


    tlv_values_t *audio_codec_params = tlv_new();
    tlv_add_integer_value(audio_codec_params, 1, 1);
    tlv_add_integer_value(audio_codec_params, 2, 0);
    tlv_add_integer_value(audio_codec_params, 3, 0);

    tlv_values_t *audio_codec = tlv_new();
    tlv_add_integer_value(audio_codec, 1, 3);
    tlv_add_tlv_value(audio_codec, 2, audio_codec_params);

    tlv_add_integer_value(&supported_rtp_config, 2, 2);  // SRTP crypto suite
}


void camera_on_resource(client_context_t *context) {
    char *body = (char*) homekit_client_get_request_body(context);
    body[homekit_client_get_request_body_size(context)] = 0;
    ESP_LOGI(TAG, "Resource payload: %s", body);

    esp_err_t err = camera_run();
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
        static unsigned char error_payload[] = "HTTP/1.1 500 Camera capture error\r\n\r\n";
        homekit_client_send(context, error_payload, sizeof(error_payload)-1);
    }

    static unsigned char success_payload[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Disposition: inline; filename=capture.jpg\r\n"
        "Content-Length: ";

    homekit_client_send(context, success_payload, sizeof(success_payload)-1);

    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%d\r\n\r\n", camera_get_data_size());

    homekit_client_send(context, (unsigned char*) buffer, strlen(buffer));
    homekit_client_send(context, camera_get_fb(), camera_get_data_size());
}


homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_ip_camera, .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "M5Stack Camera"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HaPK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "000000000001"),
            HOMEKIT_CHARACTERISTIC(MODEL, "M5Stack"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, camera_identify),
            NULL
        }),
        HOMEKIT_SERVICE(CAMERA_RTP_STREAM_MANAGEMENT, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Camera"),
            HOMEKIT_CHARACTERISTIC(STREAMING_STATUS, .getter=camera_streaming_status_get),
            HOMEKIT_CHARACTERISTIC(
                SUPPORTED_VIDEO_STREAM_CONFIGURATION,
                .value=HOMEKIT_TLV_(&supported_video_config)
            ),
            HOMEKIT_CHARACTERISTIC(
                SUPPORTED_AUDIO_STREAM_CONFIGURATION,
                .value=HOMEKIT_TLV_(&supported_audio_config)
            ),
            HOMEKIT_CHARACTERISTIC(
                SUPPORTED_RTP_CONFIGURATION,
                .value=HOMEKIT_TLV_(&supported_rtp_config)
            ),
            HOMEKIT_CHARACTERISTIC(
                SETUP_ENDPOINTS,
                .getter=camera_setup_endpoints_get,
                .setter=camera_setup_endpoints_set
            ),
            HOMEKIT_CHARACTERISTIC(
                SELECTED_RTP_STREAM_CONFIGURATION,
                .getter=camera_selected_rtp_configuration_get,
                .setter=camera_selected_rtp_configuration_set
            ),
            NULL
        }),
        NULL
    }),
    NULL
};


homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111",
    .on_resource = camera_on_resource
};


void on_wifi_ready() {
    homekit_server_init(&config);
}


void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret != ESP_OK) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK( nvs_flash_init() );
    }

    wifi_init();
    camera_accessory_init();
}
