idf_component_register(
    SRCS
        "accessory.c"
        "app_main.c"
        "camera_session.c"
        "crypto.c"
        "debug.c"
        "streaming.c"
    REQUIRES nvs_flash homekit esp32-camera libjpeg x264
)

idf_build_set_property(COMPILE_OPTIONS -UNO_SHA APPEND)
idf_build_set_property(COMPILE_OPTIONS -DWOLFSSL_AES_DIRECT APPEND)
