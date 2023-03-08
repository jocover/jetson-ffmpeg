cmake_minimum_required(VERSION 3.9)
project(nvmpi VERSION 1.0.0 DESCRIPTION "nvidia multimedia api")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS“$ {CMAKE_C_FLAGS} -fPIC”)
set(CMAKE_CXX_FLAGS“$ {CMAKE_CXX_FLAGS} -fPIC”)

set(MMAPI_PATH /usr/src/jetson_multimedia_api/)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=/lib/aarch64-linux-gnu")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=/usr/lib/aarch64-linux-gnu/tegra")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=/usr/lib/aarch64-linux-gnu")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=/usr/local/cuda/lib64")

find_package (Threads)

# file(GLOB MMAPI_SRC "${MMAPI_PATH}/samples/common/classes/*.cpp")

file(GLOB MMAPI_SRC
 ${MMAPI_PATH}/samples/common/classes/NvBuffer.cpp
 ${MMAPI_PATH}/samples/common/classes/NvElement.cpp
 ${MMAPI_PATH}/samples/common/classes/NvElementProfiler.cpp
 ${MMAPI_PATH}/samples/common/classes/NvLogging.cpp
 ${MMAPI_PATH}/samples/common/classes/NvV4l2Element.cpp
 ${MMAPI_PATH}/samples/common/classes/NvV4l2ElementPlane.cpp
 ${MMAPI_PATH}/samples/common/classes/NvVideoDecoder.cpp
 ${MMAPI_PATH}/samples/common/classes/NvVideoEncoder.cpp
 ${MMAPI_PATH}/samples/common/classes/NvBufSurface.cpp
)
add_library(nvmpi SHARED
    nvmpi_dec.cpp
    nvmpi_enc.cpp
    ${MMAPI_SRC}
  )

add_library(nvmpi_static STATIC
    nvmpi_dec.cpp
    nvmpi_enc.cpp
    ${MMAPI_SRC}
)

set(LIBS nvv4l2 nvbufsurface nvbufsurftransform)
foreach(LIB ${LIBS})
  find_library(LIB_${LIB} ${LIB} PATHS /usr/lib/aarch64-linux-gnu/tegra)
  if(LIB_${LIB})
    target_link_libraries(nvmpi PRIVATE ${LIB_${LIB}})
    target_link_libraries(nvmpi_static PRIVATE ${LIB_${LIB}})
  else()
    message("Could not find LIB_${LIB}")
  endif()
endforeach()


set_target_properties(nvmpi_static PROPERTIES OUTPUT_NAME nvmpi)
set_target_properties(nvmpi PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(nvmpi PROPERTIES SOVERSION 1)
set_target_properties(nvmpi nvmpi_static PROPERTIES PUBLIC_HEADER nvmpi.h)
target_link_libraries(nvmpi PRIVATE ${CMAKE_THREAD_LIBS_INIT} ${LIB_V4L2} ${LIB_NVBS} ${LIB_NVBST})
target_include_directories(nvmpi PRIVATE
  ${MMAPI_PATH}/include
  ${MMAPI_PATH}/include/libjpeg-8b
)
target_include_directories(nvmpi PRIVATE
  /usr/local/cuda/include
  /usr/include/libdrm
)
target_include_directories(nvmpi_static PRIVATE
  ${MMAPI_PATH}/include
  ${MMAPI_PATH}/include/libjpeg-8b
)
target_include_directories(nvmpi_static PRIVATE
  /usr/local/cuda/include
  /usr/include/libdrm
)
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
