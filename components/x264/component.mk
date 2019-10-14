# ESP_IDF
x264_THIRDPARTY_ROOT = x264-snapshot-20181221-2245-stable

COMPONENT_SRCDIRS = $(x264_THIRDPARTY_ROOT)/common $(x264_THIRDPARTY_ROOT)/encoder
COMPONENT_ADD_INCLUDEDIRS = . $(x264_THIRDPARTY_ROOT)

ifneq ($(shell $(CC) -v 2>&1 | grep 'gcc version [^0-7]'),)
CFLAGS += -Wno-error=format-truncation=
endif

$(eval $(call compile_exclude, $(addprefix $(x264_THIRDPARTY_ROOT)/, \
    common/opencl.o \
    common/threadpool.o \
    common/win32thread.o \
    encoder/rdo.o \
    encoder/slicetype.o \
)))
