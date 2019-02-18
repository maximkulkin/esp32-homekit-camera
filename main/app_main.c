#include <stdio.h>
#include <string.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_err.h>
#include <nvs_flash.h>

#include <sys/types.h>
#include <sys/socket.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include <esp_camera.h>
#include "camera.h"
#include <x264.h>
#include <jpeglib.h>

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

#define MAX_CAMERA_SESSIONS 4

#define STREAMING_STATUS_AVAILABLE 0
#define STREAMING_STATUS_IN_USE 1
#define STREAMING_STATUS_UNAVAILABLE 2


#define TAG "esp32_camera"


#define CAMERA_FRAME_RATE 30
#define CAMERA_FRAME_SIZE FRAMESIZE_VGA
#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480

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


const int camera_led_gpio = CONFIG_CAMERA_PIN_LED;
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

typedef struct {
    unsigned int cc:4;         /* CSRC count */
    unsigned int extension:1;  /* header extension flag */
    unsigned int padding:1;    /* padding flag */
    unsigned int version:2;    /* protocol version */
    unsigned int payload_type:7;  /* payload type */
    unsigned int marker:1;     /* marker bit */
    unsigned int seq:16;       /* sequence number */
    uint32_t timestamp;        /* timestamp */
    uint32_t ssrc;             /* synchronization source */
    uint32_t csrc[];           /* optional CSRC list */
} rtp_header_t;

typedef enum {
    IP_VERSION_IPV4 = 0,
    IP_VERSION_IPV6 = 1
} ip_version_t;


typedef enum {
    SRTP_CRYPTO_AES_CM_128_HMAC_SHA1_80 = 0,
    SRTP_CRYPTO_AES_256_CM_HMAC_SHA1_80 = 1,
    SRTP_CRYPTO_DISABLED = 2
} srtp_crypto_suite_t;


typedef enum {
    RTP_SESSION_END = 0,
    RTP_SESSION_START = 1,
    RTP_SESSION_SUSPEND = 2,
    RTP_SESSION_RESUME = 3,
    RTP_SESSION_RECONFIGURE = 4,
} rtp_session_command_t;


typedef enum {
    SESSION_COMMAND_END = 0,
    SESSION_COMMAND_START = 1,
    SESSION_COMMAND_SUSPEND = 2,
    SESSION_COMMAND_RESUME = 3,
    SESSION_COMMAND_RECONFIGURE = 4,
} session_command_t;

typedef struct _camera_session_t {
    homekit_client_id_t client_id;

    char session_id[17];
    uint8_t status;

    ip_version_t controller_ip_version;
    char *controller_ip_address;
    uint16_t controller_video_port;
    uint16_t controller_audio_port;

    srtp_crypto_suite_t srtp_video_crypto_suite;
    char srtp_video_master_key[33];
    size_t srtp_video_master_key_size;
    char srtp_video_master_salt[15];
    size_t srtp_video_master_salt_size;

    srtp_crypto_suite_t srtp_audio_crypto_suite;
    char srtp_audio_master_key[33];
    size_t srtp_audio_master_key_size;
    char srtp_audio_master_salt[15];
    size_t srtp_audio_master_salt_size;

    uint32_t video_ssrc;
    uint32_t audio_ssrc;

    uint8_t video_rtp_payload_type;
    uint16_t video_rtp_max_bitrate;
    float video_rtp_min_rtcp_interval;
    uint16_t video_rtp_max_mtu;

    bool active;

    int video_socket;
    uint32_t timestamp;
    uint16_t sequence;

    struct _camera_session_t *next;
} camera_session_t;



camera_session_t *camera_sessions;

camera_session_t *camera_session_new() {
    return calloc(1, sizeof(camera_session_t));
}

void camera_session_free(camera_session_t *session) {
    if (!session)
        return;

    if (session->controller_ip_address)
        free(session->controller_ip_address);

    if (session->video_socket) {
        close(session->video_socket);
    }

    free(session);
}

int camera_session_add(camera_session_t *session) {
    if (!camera_sessions) {
        camera_sessions = session;
    } else {
        camera_session_t *t = camera_sessions;
        int i = 1;
        while (t->next) {
            i++;
            t = t->next;
        }
        if (i >= MAX_CAMERA_SESSIONS)
            return -1;

        t->next = session;
    }

    return 0;
}

void camera_session_remove(camera_session_t *session) {
    if (camera_sessions == session) {
        camera_sessions = session->next;
    } else {
        camera_session_t *t = camera_sessions;
        while (t->next) {
            if (t->next == session) {
                t->next = t->next->next;
                break;
            }
            t = t->next;
        }
    }
}

camera_session_t *camera_session_find_by_client_id(homekit_client_id_t client_id) {
    camera_session_t *session = camera_sessions;
    while (session) {
        if (session->client_id == client_id)
            return session;
        session = session->next;
    }

    return NULL;
}


void camera_on_event(homekit_event_t event) {
    if (event == HOMEKIT_EVENT_CLIENT_DISCONNECTED) {
        camera_session_t *session = camera_session_find_by_client_id(homekit_get_client_id());
        if (session) {
            camera_session_remove(session);
            camera_session_free(session);
        }
    }
}


const static JOCTET EOI_BUFFER[1] = { JPEG_EOI };

typedef struct {
  struct jpeg_source_mgr pub;
  const JOCTET *data;
  size_t len;
} jpeg_memory_src_mgr;

static void jpeg_memory_src_init_source(j_decompress_ptr cinfo) {
}

static boolean jpeg_memory_src_fill_input_buffer(j_decompress_ptr cinfo) {
  jpeg_memory_src_mgr *src = (jpeg_memory_src_mgr *)cinfo->src;
  // No more data.  Probably an incomplete image;  just output EOI.
  src->pub.next_input_byte = EOI_BUFFER;
  src->pub.bytes_in_buffer = 1;
  return TRUE;
}

static void jpeg_memory_src_skip_input_data(j_decompress_ptr cinfo, long num_bytes) {
  jpeg_memory_src_mgr *src = (jpeg_memory_src_mgr *)cinfo->src;
  if (src->pub.bytes_in_buffer < num_bytes) {
    // Skipping over all of remaining data;  output EOI.
    src->pub.next_input_byte = EOI_BUFFER;
    src->pub.bytes_in_buffer = 1;
  } else {
    // Skipping over only some of the remaining data.
    src->pub.next_input_byte += num_bytes;
    src->pub.bytes_in_buffer -= num_bytes;
  }
}

static void jpeg_memory_src_term_source(j_decompress_ptr cinfo) {
}

static void jpeg_memory_src_set_source_mgr(j_decompress_ptr cinfo, const char* data, size_t len) {
  if (cinfo->src == 0) {
    cinfo->src = (struct jpeg_source_mgr *)(*cinfo->mem->alloc_small)
      ((j_common_ptr) cinfo, JPOOL_PERMANENT, sizeof(jpeg_memory_src_mgr));
  }

  jpeg_memory_src_mgr *src = (jpeg_memory_src_mgr *)cinfo->src;
  src->pub.init_source = jpeg_memory_src_init_source;
  src->pub.fill_input_buffer = jpeg_memory_src_fill_input_buffer;
  src->pub.skip_input_data = jpeg_memory_src_skip_input_data;
  src->pub.resync_to_restart = jpeg_resync_to_restart; // default
  src->pub.term_source = jpeg_memory_src_term_source;
  // fill the buffers
  src->data = (const JOCTET *)data;
  src->len = len;
  src->pub.bytes_in_buffer = len;
  src->pub.next_input_byte = src->data;
}


TaskHandle_t camera_stream_task_handle = NULL;


void camera_stream_task(void *args) {
    x264_param_t param;
    x264_param_default_preset(&param, "ultrafast", "zerolatency");

    // param.i_threads = 1;
    param.i_width = CAMERA_WIDTH;
    param.i_height = CAMERA_HEIGHT;
    param.i_bitdepth = 8;
    param.i_csp = X264_CSP_I420;
    param.i_fps_num = 2;
    param.i_fps_den = 1;
    param.i_threads = 1;

    param.i_keyint_max = 1;
    param.b_intra_refresh = 1;

    x264_param_apply_profile(&param, "baseline");

    x264_picture_t pic, pic_out;

    x264_picture_init(&pic);
    pic.img.i_csp = param.i_csp;
    pic.img.i_plane = 3;
    pic.img.i_stride[0] = param.i_width;
    pic.img.i_stride[1] = param.i_width / 2;
    pic.img.i_stride[2] = param.i_width / 2;

    const int plane_size = CAMERA_WIDTH * CAMERA_HEIGHT;
    pic.img.plane[0] = malloc(plane_size + plane_size / 2);
    pic.img.plane[1] = pic.img.plane[0] + plane_size;
    pic.img.plane[2] = pic.img.plane[1] + plane_size / 4;

    ESP_LOGI(TAG, "Initializing encoder");
    ESP_LOGI(TAG, "Total free memory: %u", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    ESP_LOGI(TAG, "Largest free block: %u", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));

    x264_t *encoder = x264_encoder_open(&param);

    uint8_t *payload = calloc(1500, 1);
    rtp_header_t *rtp_header = (rtp_header_t*)payload;
    rtp_header->version = 2;

    int frame = 0;

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGD(TAG, "Camera capture failed");
            continue;
        }

        struct jpeg_decompress_struct dinfo;
        struct jpeg_error_mgr jerr;

        dinfo.err = jpeg_std_error(&jerr);
        jpeg_create_decompress(&dinfo);
        jpeg_memory_src_set_source_mgr(&dinfo, (const char *)fb->buf, fb->len);

        jpeg_read_header(&dinfo, TRUE);

        dinfo.raw_data_out = true;
        dinfo.out_color_space = JCS_YCbCr;
        jpeg_start_decompress(&dinfo);

        #define SAMPLE_SIZE 16

        uint8_t *data_y[SAMPLE_SIZE],
                *data_cb[SAMPLE_SIZE/2],
                *data_cr[SAMPLE_SIZE/2];
        uint8_t **data[3];
        data[0] = data_y;
        data[1] = data_cb;
        data[2] = data_cr;

        for (size_t j=0; j<dinfo.output_height; ) {
            for (size_t i=0; i<SAMPLE_SIZE; i+=2) {
                data_y[i]   = pic.img.plane[0] + dinfo.image_width * (i+j);
                data_y[i+1] = pic.img.plane[0] + dinfo.image_width * (i+1+j);
                data_cb[i / 2] = pic.img.plane[1] + dinfo.image_width / 2 * ((i + j) / 2);
                data_cr[i / 2] = pic.img.plane[2] + dinfo.image_width / 2 * ((i + j) / 2);
            }

            j += jpeg_read_raw_data(&dinfo, data, SAMPLE_SIZE);
        }

        jpeg_finish_decompress(&dinfo);
        jpeg_destroy_decompress(&dinfo);

        esp_camera_fb_return(fb);

        pic.i_pts = frame;

        x264_nal_t *nal;
        int i_nal;

        ESP_LOGI(TAG, "Encoding a frame");
        ESP_LOGI(TAG, "Total free memory: %u", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
        // ESP_LOGI(TAG, "Largest free block: %u", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));

        int frame_size = x264_encoder_encode(encoder, &nal, &i_nal, &pic, &pic_out);
        if( frame_size < 0 ) {
            ESP_LOGE(TAG, "Image H264 encoding failed error = %d", frame_size);
            continue;
        } else if( frame_size ) {

            for (camera_session_t *session = camera_sessions; session; session=session->next) {
                if (!session->active)
                    continue;
                if (!session->video_socket) {
                    session->video_socket = socket(PF_INET, SOCK_DGRAM, 0);
                    if (session->video_socket < 0) {
                        ESP_LOGE(TAG, "Failed to open video stream socket, error = %d", errno);
                        continue;
                    }

                    struct sockaddr_in sin;
                    sin.sin_family = AF_INET;
                    // sin.sin_port = htons(session->controller_video_port);
                    // if (inet_pton(AF_INET, session->controller_ip_address, &sin.sin_addr) <= 0) {
                    sin.sin_port = htons(6000);
                    if (inet_pton(AF_INET, "10.0.1.5", &sin.sin_addr) <= 0) {
                        ESP_LOGE(TAG, "Failed to parse controller IP address");
                        close(session->video_socket);
                        session->video_socket = 0;
                        continue;
                    }

                    if (connect(session->video_socket, (struct sockaddr*)&sin, sizeof(sin)) < 0) {
                        ESP_LOGE(TAG, "Failed to connect to controller video port");
                        close(session->video_socket);
                        session->video_socket = 0;
                        continue;
                    }
                }

                session->timestamp++;

                rtp_header->payload_type = session->video_rtp_payload_type;
                rtp_header->ssrc = htonl(session->video_ssrc);
                rtp_header->timestamp = htonl(session->timestamp);

                size_t sent_size = 0;
                while (sent_size < frame_size) {
                    size_t packet_size = MIN(frame_size, session->video_rtp_max_mtu);

                    rtp_header->seq = htonl(session->sequence++);
                    memcpy(payload + sizeof(rtp_header), nal->p_payload, packet_size);
                    if (send(session->video_socket, payload, packet_size + sizeof(rtp_header), 0) < 0) {
                        ESP_LOGE(TAG, "Failed to send RTP packet");
                        break;
                    }

                    sent_size += packet_size;
                }
            }
        }

        frame++;
    }

    x264_encoder_close(encoder);

    free(pic.img.plane[0]);

    ESP_LOGI(TAG, "Done with stream");
    ESP_LOGI(TAG, "Total free memory: %u", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    ESP_LOGI(TAG, "Largest free block: %u", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
}


bool camera_stream_task_running() {
    return camera_stream_task_handle != NULL;
}

void camera_stream_task_start() {
    xTaskCreate(camera_stream_task, "Camera Stream",
                4096*8, NULL, 1, &camera_stream_task_handle);
}

void camera_stream_task_stop() {
    vTaskDelete(camera_stream_task_handle);
}


homekit_value_t camera_streaming_status_get() {
    tlv_values_t *tlv = tlv_new();
    tlv_add_integer_value(tlv, 1, 1, STREAMING_STATUS_AVAILABLE);
    return HOMEKIT_TLV(tlv);
}

homekit_value_t camera_setup_endpoints_get() {
    ESP_LOGI(TAG, "Creating setup endpoints response");
    homekit_client_id_t client_id = homekit_get_client_id();
    if (!client_id) {
        ESP_LOGI(TAG, "No client found");
        return HOMEKIT_TLV(tlv_new());
    }

    camera_session_t *session = camera_session_find_by_client_id(client_id);
    if (!session) {
        return HOMEKIT_TLV(tlv_new());
    }

    tlv_values_t *controller_address = tlv_new();
    tlv_add_integer_value(controller_address, 1, 1, session->controller_ip_version);
    tlv_add_string_value(controller_address, 2, session->controller_ip_address);
    tlv_add_integer_value(controller_address, 3, 2, session->controller_video_port);
    tlv_add_integer_value(controller_address, 4, 2, session->controller_audio_port);

    tlv_values_t *video_rtp_params = tlv_new();
    tlv_add_integer_value(video_rtp_params, 1, 1, session->srtp_video_crypto_suite);
    tlv_add_value(video_rtp_params, 2, (unsigned char*)session->srtp_video_master_key, session->srtp_video_master_key_size);
    tlv_add_value(video_rtp_params, 3, (unsigned char*)session->srtp_video_master_salt, session->srtp_video_master_salt_size);

    tlv_values_t *audio_rtp_params = tlv_new();
    tlv_add_integer_value(audio_rtp_params, 1, 1, session->srtp_audio_crypto_suite);
    tlv_add_value(audio_rtp_params, 2, (unsigned char*)session->srtp_audio_master_key, session->srtp_audio_master_key_size);
    tlv_add_value(audio_rtp_params, 3, (unsigned char*)session->srtp_audio_master_salt, session->srtp_audio_master_salt_size);

    tlv_values_t *response = tlv_new();
    tlv_add_value(response, 1, (unsigned char*)session->session_id, 16);
    tlv_add_integer_value(response, 2, 1, session->status);
    tlv_add_tlv_value(response, 3, controller_address);
    tlv_add_tlv_value(response, 4, video_rtp_params);
    tlv_add_tlv_value(response, 5, audio_rtp_params);
    tlv_add_integer_value(response, 6, 4, session->video_ssrc);
    tlv_add_integer_value(response, 7, 4, session->audio_ssrc);

    tlv_free(controller_address);
    tlv_free(video_rtp_params);
    tlv_free(audio_rtp_params);

    return HOMEKIT_TLV(response);
}


void camera_setup_endpoints_set(homekit_value_t value) {
    if (value.format != homekit_format_tlv) {
        ESP_LOGE(TAG, "Invalid value format: %d", value.format);
        return;
    }

    #define error_msg "Failed to setup endpoints: "

    homekit_client_id_t client_id = homekit_get_client_id();

    camera_session_t *session = camera_session_find_by_client_id(client_id);
    if (!session) {
        session = camera_session_new();
        session->client_id = client_id;
    }

    tlv_values_t *request = value.tlv_values;

    tlv_t *v;
    int x;

    v = tlv_get_value(request, 1);
    if (!v) {
        ESP_LOGE(TAG, error_msg "no session ID field");
        camera_session_free(session);
        return;
    }

    if (v->size != 16) {
        ESP_LOGE(TAG, error_msg "session ID field has invalid size (%d)", v->size);
        camera_session_free(session);
        return;
    }
    memcpy(session->session_id, (char*)v->value, v->size);
    session->session_id[v->size] = 0;

    tlv_values_t *controller_address = tlv_get_tlv_value(request, 3);
    if (!controller_address) {
        ESP_LOGE(TAG, error_msg "no controller address field");
        camera_session_free(session);
        return;
    }

    x = tlv_get_integer_value(controller_address, 1, -1);
    if (x == -1) {
        ESP_LOGE(TAG, error_msg "no controller IP address version field");
        tlv_free(controller_address);
        camera_session_free(session);
        return;
    }
    session->controller_ip_version = x;

    v = tlv_get_value(controller_address, 2);
    if (!v) {
        ESP_LOGE(TAG, error_msg "no controller IP address field");
        tlv_free(controller_address);
        camera_session_free(session);
        return;
    }
    char *ip_address = strndup((char*)v->value, v->size);
    session->controller_ip_address = ip_address;

    x = tlv_get_integer_value(controller_address, 3, -1);
    if (x == -1) {
        ESP_LOGE(TAG, error_msg "no controller video port field");
        tlv_free(controller_address);
        camera_session_free(session);
        return;
    }
    session->controller_video_port = x;

    x = tlv_get_integer_value(controller_address, 4, -1);
    if (x == -1) {
        ESP_LOGE(TAG, error_msg "no controller audio port field");
        tlv_free(controller_address);
        camera_session_free(session);
        return;
    }
    session->controller_audio_port = x;

    tlv_free(controller_address);

    tlv_values_t *rtp_params = tlv_get_tlv_value(request, 4);
    if (!rtp_params) {
        ESP_LOGE(TAG, error_msg "no video RTP params field");
        camera_session_free(session);
        return;
    }

    x = tlv_get_integer_value(rtp_params, 1, -1);
    if (x == -1) {
        ESP_LOGE(TAG, error_msg "no video RTP params crypto suite field");
        tlv_free(rtp_params);
        camera_session_free(session);
        return;
    }
    session->srtp_video_crypto_suite = x;

    v = tlv_get_value(rtp_params, 2);
    if (!v) {
        ESP_LOGE(TAG, error_msg "no video RTP params master key field");
        tlv_free(rtp_params);
        camera_session_free(session);
        return;
    }
    if (v->size >= sizeof(session->srtp_video_master_key)) {
        ESP_LOGE(TAG, error_msg "invalid video RTP params master key size (%d)", v->size);
        tlv_free(rtp_params);
        camera_session_free(session);
        return;
    }
    memcpy(session->srtp_video_master_key, (char*)v->value, v->size);
    session->srtp_video_master_key_size = v->size;

    v = tlv_get_value(rtp_params, 3);
    if (!v) {
        ESP_LOGE(TAG, error_msg "no video RTP params master salt field");
        tlv_free(rtp_params);
        camera_session_free(session);
        return;
    }
    if (v->size >= sizeof(session->srtp_video_master_salt)) {
        ESP_LOGE(TAG, error_msg "invalid video RTP params master key size (%d)", v->size);
        tlv_free(rtp_params);
        camera_session_free(session);
        return;
    }
    memcpy(session->srtp_video_master_salt, (char*)v->value, v->size);
    session->srtp_video_master_salt_size = v->size;

    tlv_free(rtp_params);

    rtp_params = tlv_get_tlv_value(request, 5);
    if (!rtp_params) {
        ESP_LOGE(TAG, error_msg "no audio RTP params field");
        camera_session_free(session);
        return;
    }

    x = tlv_get_integer_value(rtp_params, 1, -1);
    if (x == -1) {
        ESP_LOGE(TAG, error_msg "no audio RTP params crypto suite field");
        tlv_free(rtp_params);
        camera_session_free(session);
        return;
    }
    session->srtp_audio_crypto_suite = x;

    v = tlv_get_value(rtp_params, 2);
    if (!v) {
        ESP_LOGE(TAG, error_msg "no audio RTP params master key field");
        tlv_free(rtp_params);
        camera_session_free(session);
        return;
    }
    if (v->size >= sizeof(session->srtp_audio_master_key)) {
        ESP_LOGE(TAG, error_msg "invalid audio RTP params master key size (%d)", v->size);
        tlv_free(rtp_params);
        camera_session_free(session);
        return;
    }
    memcpy(session->srtp_audio_master_key, (char*)v->value, v->size);
    session->srtp_audio_master_key_size = v->size;

    v = tlv_get_value(rtp_params, 3);
    if (!v) {
        ESP_LOGE(TAG, error_msg "no audio RTP params master salt field");
        tlv_free(rtp_params);
        camera_session_free(session);
        return;
    }
    if (v->size >= sizeof(session->srtp_audio_master_salt)) {
        ESP_LOGE(TAG, error_msg "invalid audio RTP params master salt size (%d)", v->size);
        tlv_free(rtp_params);
        camera_session_free(session);
        return;
    }
    memcpy(session->srtp_audio_master_salt, (char*)v->value, v->size);
    session->srtp_audio_master_salt_size = v->size;

    tlv_free(rtp_params);

    #undef error_msg

    if (camera_session_add(session)) {
        // session registration failed
        session->status = 2;
    }
}

homekit_value_t camera_selected_rtp_configuration_get() {
    return HOMEKIT_TLV(tlv_new());
}

void camera_selected_rtp_configuration_set(homekit_value_t value) {
    if (value.format != homekit_format_tlv) {
        ESP_LOGE(TAG, "Failed to setup selected RTP config: invalid value format: %d", value.format);
        return;
    }

    #define error_msg "Failed to setup selected RTP config: %s"

    camera_session_t *session = camera_session_find_by_client_id(homekit_get_client_id());
    if (!session)
        return;

    tlv_values_t *request = value.tlv_values;
    int x;

    tlv_values_t *session_control = tlv_get_tlv_value(request, 1);
    if (!session_control) {
        ESP_LOGE(TAG, error_msg, "no session control field");
        return;
    }
    tlv_t *session_id = tlv_get_value(session_control, 1);
    if (!session_id) {
        ESP_LOGE(TAG, error_msg, "no session ID field");
        tlv_free(session_control);
        return;
    }
    // TODO: find session with given ID
    if (strncmp(session->session_id, (char*)session_id->value, sizeof(session->session_id)-1)) {
        ESP_LOGE(TAG, error_msg, "invalid session ID");
        tlv_free(session_control);
        return;
    }

    int session_command = tlv_get_integer_value(session_control, 2, -1);
    if (session_command == -1) {
        ESP_LOGE(TAG, error_msg, "no command field");
        tlv_free(session_control);
        return;
    }
    tlv_free(session_control);

    if (session_command == RTP_SESSION_START || session_command == RTP_SESSION_RECONFIGURE) {
        tlv_values_t *selected_video_params = tlv_get_tlv_value(request, 2);
        if (!selected_video_params) {
            ESP_LOGE(TAG, error_msg, "no selected video params field");
            return;
        }

        tlv_values_t *video_rtp_params = tlv_get_tlv_value(selected_video_params, 4);
        if (!video_rtp_params) {
            ESP_LOGE(TAG, error_msg, "no selected video RTP params field");
            tlv_free(selected_video_params);
            return;
        }

        x = tlv_get_integer_value(video_rtp_params, 1, -1);
        if (x == -1) {
            ESP_LOGE(TAG, error_msg, "no selected video RTP payload type field");
            tlv_free(video_rtp_params);
            tlv_free(selected_video_params);
            return;
        }
        session->video_rtp_payload_type = x;

        x = tlv_get_integer_value(video_rtp_params, 2, 0);
        if (x == 0) {
            ESP_LOGE(TAG, error_msg, "no selected video RTP SSRC field");
            tlv_free(video_rtp_params);
            tlv_free(selected_video_params);
            return;
        }
        session->video_ssrc = x;

        x = tlv_get_integer_value(video_rtp_params, 3, -1);
        if (x == -1) {
            ESP_LOGE(TAG, error_msg, "no selected video RTP max bitrate field");
            tlv_free(video_rtp_params);
            tlv_free(selected_video_params);
            return;
        }
        session->video_rtp_max_bitrate = x;

        // TODO: parse min RTCP interval

        x = tlv_get_integer_value(video_rtp_params, 5, -1);
        if (x == -1) {
            ESP_LOGE(TAG, error_msg, "no selected video RTP max MTU field");
            tlv_free(video_rtp_params);
            tlv_free(selected_video_params);
            return;
        }
        session->video_rtp_max_mtu = x;

        tlv_free(video_rtp_params);
        tlv_free(selected_video_params);
    }

    // TODO: process command
    switch (session_command) {
        case SESSION_COMMAND_START:
        case SESSION_COMMAND_RESUME:
        case SESSION_COMMAND_RECONFIGURE:
            session->active = true;
            if (!camera_stream_task_running()) {
                camera_stream_task_start();
            }

            break;
        case SESSION_COMMAND_SUSPEND:
        case SESSION_COMMAND_END: {
            session->active = false;

            if (session->video_socket) {
                close(session->video_socket);
                session->video_socket = 0;
            }

            bool streams_active = false;
            camera_session_t *s = camera_sessions;
            while (s) {
                if (s->active) {
                    streams_active = true;
                    break;
                }
                s = s->next;
            }
            if (!streams_active && camera_stream_task_running()) {
                camera_stream_task_stop();
            }
            break;
        }
    }

    #undef error_msg
}

tlv_values_t supported_video_config = {};
tlv_values_t supported_audio_config = {};
tlv_values_t supported_rtp_config = {};


void camera_accessory_init() {
    ESP_LOGI(TAG, "Free heap: %d", xPortGetFreeHeapSize());

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    gpio_set_direction(camera_led_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(camera_led_gpio, 1);

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("camera", ESP_LOG_VERBOSE);

    /* IO13, IO14 is designed for JTAG by default,
     * to use it as generalized input,
     * firstly declair it as pullup input */
    gpio_config_t conf;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.pin_bit_mask = 1LL << 13;
    gpio_config(&conf);
    conf.pin_bit_mask = 1LL << 14;
    gpio_config(&conf);

    camera_config_t camera_config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pin_d0 = CAMERA_PIN_D0,
        .pin_d1 = CAMERA_PIN_D1,
        .pin_d2 = CAMERA_PIN_D2,
        .pin_d3 = CAMERA_PIN_D3,
        .pin_d4 = CAMERA_PIN_D4,
        .pin_d5 = CAMERA_PIN_D5,
        .pin_d6 = CAMERA_PIN_D6,
        .pin_d7 = CAMERA_PIN_D7,
        .pin_xclk = CAMERA_PIN_XCLK,
        .pin_pclk = CAMERA_PIN_PCLK,
        .pin_vsync = CAMERA_PIN_VSYNC,
        .pin_href = CAMERA_PIN_HREF,
        .pin_sscb_sda = CAMERA_PIN_SIOD,
        .pin_sscb_scl = CAMERA_PIN_SIOC,
        .pin_pwdn = CAMERA_PIN_PWDN,
        .pin_reset = CAMERA_PIN_RESET,
        .xclk_freq_hz = CONFIG_XCLK_FREQ,

        .frame_size = CAMERA_FRAME_SIZE,
        .pixel_format = PIXFORMAT_JPEG,
        .jpeg_quality = 15,

        .fb_count = 2,
    };

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    tlv_values_t *video_codec_params = tlv_new();
    tlv_add_integer_value(video_codec_params, 1, 1, 1);  // Profile ID
    tlv_add_integer_value(video_codec_params, 2, 1, 0);  // Level
    tlv_add_integer_value(video_codec_params, 3, 1, 0);  // Packetization mode

    tlv_values_t *video_attributes = tlv_new();
    tlv_add_integer_value(video_attributes, 1, 2, CAMERA_WIDTH);  // Image width
    tlv_add_integer_value(video_attributes, 2, 2, CAMERA_HEIGHT);  // Image height
    tlv_add_integer_value(video_attributes, 3, 2, CAMERA_FRAME_RATE);  // Frame rate

    tlv_values_t *video_codec_config = tlv_new();
    tlv_add_integer_value(video_codec_config, 1, 1, 0);  // Video codec type
    tlv_add_tlv_value(video_codec_config, 2, video_codec_params);  // Video codec params
    tlv_add_tlv_value(video_codec_config, 3, video_attributes);  // Video attributes

    tlv_add_tlv_value(&supported_video_config, 1, video_codec_config);

    tlv_free(video_codec_config);
    tlv_free(video_attributes);
    tlv_free(video_codec_params);

    tlv_values_t *audio_codec_params = tlv_new();
    tlv_add_integer_value(audio_codec_params, 1, 1, 1);  // Number of audio channels
    tlv_add_integer_value(audio_codec_params, 2, 1, 0);  // Bit-rate
    tlv_add_integer_value(audio_codec_params, 3, 1, 2);  // Sample rate

    tlv_values_t *audio_codec = tlv_new();
    tlv_add_integer_value(audio_codec, 1, 1, 3);
    tlv_add_tlv_value(audio_codec, 2, audio_codec_params);

    tlv_add_tlv_value(&supported_audio_config, 1, audio_codec);
    tlv_add_integer_value(&supported_audio_config, 2, 1, 0);  // Comfort noise support

    tlv_free(audio_codec);
    tlv_free(audio_codec_params);

    tlv_add_integer_value(&supported_rtp_config, 2, 1, 0);  // SRTP crypto suite

    camera_sessions = NULL;
}


void camera_on_resource(const char *body, size_t body_size) {
    ESP_LOGI(TAG, "Resource payload: %s", body);
    // cJSON *json = cJSON_Parse(body);
    // process json
    // cJSON_Delete(json);

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGD(TAG, "Camera capture failed");
        static unsigned char error_payload[] = "HTTP/1.1 500 Camera capture error\r\n\r\n";
        homekit_client_send(error_payload, sizeof(error_payload)-1);
        return;
    }

    static unsigned char success_payload[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Disposition: inline; filename=capture.jpg\r\n"
        "Content-Length: ";

    homekit_client_send(success_payload, sizeof(success_payload)-1);

    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%d\r\n\r\n", fb->len);

    homekit_client_send((unsigned char*) buffer, strlen(buffer));
    homekit_client_send(fb->buf, fb->len);

    esp_camera_fb_return(fb);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverride-init"
homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1,
                      .category=homekit_accessory_category_ip_camera,
                      .config_number=3,
                      .services=(homekit_service_t*[])
    {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "M5Stack Camera"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HaPK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "1"),
            HOMEKIT_CHARACTERISTIC(MODEL, "M5Stack"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, camera_identify),
            NULL
        }),
        HOMEKIT_SERVICE(CAMERA_RTP_STREAM_MANAGEMENT, .primary=true, .characteristics=(homekit_characteristic_t*[]){
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
            HOMEKIT_CHARACTERISTIC(STREAMING_STATUS, .getter=camera_streaming_status_get),
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
        HOMEKIT_SERVICE(MICROPHONE, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(VOLUME, 0),
            HOMEKIT_CHARACTERISTIC(MUTE, false),
            NULL
        }),
        NULL
    }),
    NULL
};
#pragma GCC diagnostic pop


homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111",
    .on_event = camera_on_event,
    .on_resource = camera_on_resource,
};


void on_wifi_ready() {
    camera_accessory_init();
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
}
