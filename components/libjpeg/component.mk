# ESP_IDF
libjpeg_THIRDPARTY_ROOT = jpeg-6b

COMPONENT_SRCDIRS = $(libjpeg_THIRDPARTY_ROOT)
COMPONENT_ADD_INCLUDEDIRS = . $(libjpeg_THIRDPARTY_ROOT)

$(eval $(call compile_exclude, $(addprefix $(libjpeg_THIRDPARTY_ROOT)/, \
    ansi2knr.o \
    example.o \
    jmemname.o \
    jmemdos.o \
    jmemmac.o \
)))
