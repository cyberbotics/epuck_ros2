# Build test examples for e-puck2 camera
# - `v4l2_to_rgb24.cpp` reads from V4L2 and convert to JPEG and RGB (`cmake .`)
# - `cv_to_jpeg.cpp` reads from file using OpenCV and converts to JPEG (`cmake -DBUILD_OPENCV_TEST:BOOL=true .`)

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
  ../src/pipuck_mmal.c
  ../src/pipuck_ov7670.c
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

if(BUILD_OPENCV_TEST)
  find_package(OpenCV REQUIRED)

  add_executable(cv_to_jpeg
    cv_to_jpeg.cpp
    ../src/pipuck_v4l2.c
    ../src/pipuck_mmal.c
    ../src/pipuck_ov7670.c
  )

  target_include_directories(cv_to_jpeg PUBLIC
    ../include
    /opt/vc/include
    /opt/vc/include/interface/vcos/pthreads
    /opt/vc/include/interface/vmcs_host/linux
  )

  target_link_libraries(cv_to_jpeg
    ${MMAL_LIBS}
    ${OpenCV_LIBS}
  )
endif()
