# ESP_IDF
libswscale_THIRDPARTY_ROOT = libswscale
libavutil_THIRDPARTY_ROOT = libavutil

COMPONENT_SRCDIRS = $(libswscale_THIRDPARTY_ROOT) $(libavutil_THIRDPARTY_ROOT)
COMPONENT_ADD_INCLUDEDIRS = . $(libswscale_THIRDPARTY_ROOT)

$(eval $(call compile_exclude, $(addprefix $(libswscale_THIRDPARTY_ROOT)/, \
    bayer_template.o \
    rgb2rgb_template.o \
)))

$(eval $(call compile_exclude, $(addprefix $(libavutil_THIRDPARTY_ROOT)/, \
    hwcontext_cuda.o cuda_check.o \
    hwcontext_d3d11va.o \
    hwcontext_dxva2.o \
    hwcontext_drm.o \
    hwcontext_mediacodec.o \
    hwcontext_opencl.o \
    hwcontext_qsv.o \
    hwcontext_vaapi.o \
    hwcontext_videotoolbox.o \
    hwcontext_vdpau.o \
    avutilres.o \
    fixed_dsp.o \
)))
