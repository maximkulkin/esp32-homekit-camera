#pragma once

#if CONFIG_CAMERA_MODEL_WROVER_KIT
#define CAMERA_PIN_PWDN    -1
#define CAMERA_PIN_RESET   -1
#define CAMERA_PIN_SIOD    26
#define CAMERA_PIN_SIOC    27
#define CAMERA_PIN_XCLK    21

#define CAMERA_PIN_D0       4
#define CAMERA_PIN_D1       5
#define CAMERA_PIN_D2      18
#define CAMERA_PIN_D3      19
#define CAMERA_PIN_D4      36
#define CAMERA_PIN_D5      39
#define CAMERA_PIN_D6      34
#define CAMERA_PIN_D7      35
#define CAMERA_PIN_VSYNC   25
#define CAMERA_PIN_HREF    23
#define CAMERA_PIN_PCLK    22

#elif CONFIG_CAMERA_MODEL_ESP_EYE
#define CAMERA_PIN_PWDN    -1
#define CAMERA_PIN_RESET   -1
#define CAMERA_PIN_SIOD    18
#define CAMERA_PIN_SIOC    23
#define CAMERA_PIN_XCLK    4

#define CAMERA_PIN_D0      34
#define CAMERA_PIN_D1      13
#define CAMERA_PIN_D2      14
#define CAMERA_PIN_D3      35
#define CAMERA_PIN_D4      39
#define CAMERA_PIN_D5      38
#define CAMERA_PIN_D6      37
#define CAMERA_PIN_D7      36
#define CAMERA_PIN_VSYNC   5
#define CAMERA_PIN_HREF    27
#define CAMERA_PIN_PCLK    25

#elif CONFIG_CAMERA_MODEL_M5STACK_PSRAM
#define CAMERA_PIN_PWDN     -1
#define CAMERA_PIN_RESET    15
#define CAMERA_PIN_SIOD     22
#define CAMERA_PIN_SIOC     23
#define CAMERA_PIN_XCLK     27

#define CAMERA_PIN_D0       32
#define CAMERA_PIN_D1       35
#define CAMERA_PIN_D2       34
#define CAMERA_PIN_D3        5
#define CAMERA_PIN_D4       39
#define CAMERA_PIN_D5       18
#define CAMERA_PIN_D6       36
#define CAMERA_PIN_D7       19
#define CAMERA_PIN_VSYNC    25
#define CAMERA_PIN_HREF     26
#define CAMERA_PIN_PCLK     21

#elif CONFIG_CAMERA_MODEL_M5STACK
#define CAMERA_PIN_PWDN     -1
#define CAMERA_PIN_RESET    15
#define CAMERA_PIN_SIOD     25
#define CAMERA_PIN_SIOC     23
#define CAMERA_PIN_XCLK     27

#define CAMERA_PIN_D0       17
#define CAMERA_PIN_D1       35
#define CAMERA_PIN_D2       34
#define CAMERA_PIN_D3        5
#define CAMERA_PIN_D4       39
#define CAMERA_PIN_D5       18
#define CAMERA_PIN_D6       36
#define CAMERA_PIN_D7       19
#define CAMERA_PIN_VSYNC    22
#define CAMERA_PIN_HREF     26
#define CAMERA_PIN_PCLK     21

#elif CONFIG_CAMERA_MODEL_AI_THINKER
#define CAMERA_PIN_PWDN     32
#define CAMERA_PIN_RESET    -1
#define CAMERA_PIN_SIOD     26
#define CAMERA_PIN_SIOC     27
#define CAMERA_PIN_XCLK      0

#define CAMERA_PIN_D0        5
#define CAMERA_PIN_D1       18
#define CAMERA_PIN_D2       19
#define CAMERA_PIN_D3       21
#define CAMERA_PIN_D4       36
#define CAMERA_PIN_D5       39
#define CAMERA_PIN_D6       34
#define CAMERA_PIN_D7       35
#define CAMERA_PIN_VSYNC    25
#define CAMERA_PIN_HREF     23
#define CAMERA_PIN_PCLK     22

#elif CONFIG_CAMERA_MODEL_TTGO_V1_7
#define CAMERA_PIN_PWDN     26
#define CAMERA_PIN_RESET    -1
#define CAMERA_PIN_SIOD     13
#define CAMERA_PIN_SIOC     12
#define CAMERA_PIN_XCLK     32

#define CAMERA_PIN_D0        5
#define CAMERA_PIN_D1       14
#define CAMERA_PIN_D2        4
#define CAMERA_PIN_D3       15
#define CAMERA_PIN_D4       18
#define CAMERA_PIN_D5       23
#define CAMERA_PIN_D6       36
#define CAMERA_PIN_D7       39
#define CAMERA_PIN_VSYNC    27
#define CAMERA_PIN_HREF     25
#define CAMERA_PIN_PCLK     19

#elif CONFIG_CAMERA_MODEL_DIYMORE
#define CAMERA_PIN_PWDN     32
#define CAMERA_PIN_RESET     2
#define CAMERA_PIN_SIOD     26
#define CAMERA_PIN_SIOC     27
#define CAMERA_PIN_XCLK      0

#define CAMERA_PIN_D0        5
#define CAMERA_PIN_D1       18
#define CAMERA_PIN_D2       19
#define CAMERA_PIN_D3       21
#define CAMERA_PIN_D4       36
#define CAMERA_PIN_D5       39
#define CAMERA_PIN_D6       34
#define CAMERA_PIN_D7       35
#define CAMERA_PIN_VSYNC    25
#define CAMERA_PIN_HREF     23
#define CAMERA_PIN_PCLK     22


#elif CONFIG_CAMERA_MODEL_CUSTOM
#define CAMERA_PIN_PWDN    CONFIG_CAMERA_PIN_PWDN
#define CAMERA_PIN_RESET   CONFIG_CAMERA_PIN_RESET
#define CAMERA_PIN_XCLK    CONFIG_CAMERA_PIN_XCLK
#define CAMERA_PIN_SIOD    CONFIG_CAMERA_PIN_SIOD
#define CAMERA_PIN_SIOC    CONFIG_CAMERA_PIN_SIOC

#define CAMERA_PIN_D0      CONFIG_CAMERA_PIN_D0
#define CAMERA_PIN_D1      CONFIG_CAMERA_PIN_D1
#define CAMERA_PIN_D2      CONFIG_CAMERA_PIN_D2
#define CAMERA_PIN_D3      CONFIG_CAMERA_PIN_D3
#define CAMERA_PIN_D4      CONFIG_CAMERA_PIN_D4
#define CAMERA_PIN_D5      CONFIG_CAMERA_PIN_D5
#define CAMERA_PIN_D6      CONFIG_CAMERA_PIN_D6
#define CAMERA_PIN_D7      CONFIG_CAMERA_PIN_D7
#define CAMERA_PIN_VSYNC   CONFIG_CAMERA_PIN_VSYNC
#define CAMERA_PIN_HREF    CONFIG_CAMERA_PIN_HREF
#define CAMERA_PIN_PCLK    CONFIG_CAMERA_PIN_PCLK
#endif

