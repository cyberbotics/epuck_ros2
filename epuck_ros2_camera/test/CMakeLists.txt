cmake_minimum_required(VERSION 3.5)
project(pipuck_tests)

set(CMAKE_CXX_STANDARD 14)

set(MMAL_LIBS
  pthread
  /opt/vc/lib/libbcm_host.so
  /opt/vc/lib/libvcos.so
  /opt/vc/lib/libmmal.so
  /opt/vc/lib/libmmal_core.so
  /opt/vc/lib/libmmal_util.so
  /opt/vc/lib/libmmal_vc_client.so
  /opt/vc/lib/libvchostif.a
)

add_executable(v4l2_to_rgb24
  v4l2_to_rgb24.cpp
  ../src/pipuck_v4l2.c
  ../src/pipuck_jpeg.c
  ../src/pipuck_ov7670.c
  ../src/pipuck_image.c
)

target_include_directories(v4l2_to_rgb24 PUBLIC
    ../include
  /opt/vc/include
  /opt/vc/include/interface/vcos/pthreads
  /opt/vc/include/interface/vmcs_host/linux
)

target_link_libraries(v4l2_to_rgb24
  ${MMAL_LIBS}
)