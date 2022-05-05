file(REAL_PATH "x264-snapshot-20181221-2245-stable" x264_THIRDPARTY_ROOT)

idf_component_register(
    SRC_DIRS
        "${x264_THIRDPARTY_ROOT}/common"
        "${x264_THIRDPARTY_ROOT}/encoder"
    EXCLUDE_SRCS
        "${x264_THIRDPARTY_ROOT}/common/opencl.c" 
        "${x264_THIRDPARTY_ROOT}/common/threadpool.c"
        "${x264_THIRDPARTY_ROOT}/common/win32thread.c"
        "${x264_THIRDPARTY_ROOT}/encoder/rdo.c"
        "${x264_THIRDPARTY_ROOT}/encoder/slicetype.c"
    INCLUDE_DIRS "." "${x264_THIRDPARTY_ROOT}"
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=format-truncation=
    -DBIT_DEPTH=8
    -DHIGH_BIT_DEPTH=0
)
