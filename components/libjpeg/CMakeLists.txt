file(REAL_PATH "jpeg-6b" libjpeg_THIRDPARTY_ROOT)

idf_component_register(
    SRC_DIRS "${libjpeg_THIRDPARTY_ROOT}"
    EXCLUDE_SRCS
        "${libjpeg_THIRDPARTY_ROOT}/ansi2knr.c"
        "${libjpeg_THIRDPARTY_ROOT}/example.c"
        "${libjpeg_THIRDPARTY_ROOT}/jmemname.c"
        "${libjpeg_THIRDPARTY_ROOT}/jmemdos.c"
        "${libjpeg_THIRDPARTY_ROOT}/jmemmac.c"
    INCLUDE_DIRS "." "${libjpeg_THIRDPARTY_ROOT}"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=main)
