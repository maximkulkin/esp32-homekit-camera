#pragma once

// Configuration stuff

#define TAG "esp32_camera"

#define CAMERA_FRAME_RATE 30

// frame sizes based on components/esp32-camera/driver/include/sensor.h
//
// FRAMESIZE_XGA   - 1024x768
// FRAMESIZE_SVGA  -  800x600
// FRAMESIZE_VGA   -  640x480
// FRAMESIZE_QVGA  -  320x240

// Captured image frame size (same size used for snapshots)
#define CAMERA_FRAME_SIZE FRAMESIZE_VGA

// Scale factors to get image for video stream
#define VIDEO_IMAGE_SCALE_NUM 1
#define VIDEO_IMAGE_SCALE_DENOM 8

// Calculated stuff

#if (CAMERA_FRAME_SIZE == FRAMESIZE_XGA)
#define CAMERA_WIDTH 1024
#define CAMERA_HEIGHT 768

#elif (CAMERA_FRAME_SIZE == FRAMESIZE_SVGA)
#define CAMERA_WIDTH 800
#define CAMERA_HEIGHT 600

#elif (CAMERA_FRAME_SIZE == FRAMESIZE_VGA)
#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480

#elif (CAMERA_FRAME_SIZE == FRAMESIZE_QVGA)
#define CAMERA_WIDTH 320
#define CAMERA_HEIGHT 240

#else
#error Unsupported camera frame size

#endif

#define VIDEO_IMAGE_SCALED(x) ((x) * VIDEO_IMAGE_SCALE_NUM / VIDEO_IMAGE_SCALE_DENOM)

#define VIDEO_WIDTH VIDEO_IMAGE_SCALED(CAMERA_WIDTH)
#define VIDEO_HEIGHT VIDEO_IMAGE_SCALED(CAMERA_HEIGHT)
