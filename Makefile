PROJECT_NAME := esp32-homekit-camera

CFLAGS += -DHOMEKIT_SHORT_APPLE_UUIDS

include $(IDF_PATH)/make/project.mk
