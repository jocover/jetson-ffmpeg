cmake_minimum_required(VERSION 3.9)
project(nvmpi VERSION 1.0.0 DESCRIPTION "nvidia multimedia api")

set(CMAKE_C_FLAGS“$ {CMAKE_C_FLAGS} -fPIC”)
set(CMAKE_CXX_FLAGS“$ {CMAKE_CXX_FLAGS} -fPIC”)
#set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=/lib")
#set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=/usr/lib/aarch64-linux-gnu")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=/usr/lib/aarch64-linux-gnu/tegra")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=/usr/local/cuda/lib64")

find_library(LIB_NVBUF nvbuf_utils PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(LIB_V4L2 nvv4l2 PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_package (Threads)
#find_library(LIB_DRM drm PATHS /usr/lib/aarch64-linux-gnu/tegra)
#find_library(LIB_EGL EGL PATHS /usr/lib/aarch64-linux-gnu/tegra)

add_library(nvmpi SHARED
    nvmpi_dec.cpp
    nvmpi_enc.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvBuffer.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvElement.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvElementProfiler.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvLogging.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvV4l2Element.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvV4l2ElementPlane.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvVideoDecoder.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvVideoEncoder.cpp
    #common/NvVideoConverter.cpp
    #common/NvApplicationProfiler.cpp
    #common/NvEglRenderer.cpp
    #common/NvUtils.cpp
)

add_library(nvmpi_static STATIC
    nvmpi_dec.cpp
    nvmpi_enc.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvBuffer.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvElement.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvElementProfiler.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvLogging.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvV4l2Element.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvV4l2ElementPlane.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvVideoDecoder.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvVideoEncoder.cpp
    #common/NvVideoConverter.cpp
    #common/NvApplicationProfiler.cpp
    #common/NvEglRenderer.cpp
    #common/NvUtils.cpp
)

set_target_properties(nvmpi_static PROPERTIES OUTPUT_NAME nvmpi)
set_target_properties(nvmpi PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(nvmpi PROPERTIES SOVERSION 1)
set_target_properties(nvmpi nvmpi_static PROPERTIES PUBLIC_HEADER nvmpi.h)
target_link_libraries(nvmpi PRIVATE ${CMAKE_THREAD_LIBS_INIT} ${LIB_NVBUF}  ${LIB_V4L2})
target_include_directories(nvmpi PRIVATE /usr/src/jetson_multimedia_api/include)
target_include_directories(nvmpi PRIVATE /usr/local/cuda/include)
target_include_directories(nvmpi_static PRIVATE /usr/src/jetson_multimedia_api/include)
target_include_directories(nvmpi_static PRIVATE /usr/local/cuda/include)
configure_file(nvmpi.pc.in nvmpi.pc @ONLY)
include(GNUInstallDirs)
install(TARGETS nvmpi nvmpi_static
	    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${CMAKE_BINARY_DIR}/nvmpi.pc
	    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)
install(FILES ${CMAKE_BINARY_DIR}/nvmpi.pc
	    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
