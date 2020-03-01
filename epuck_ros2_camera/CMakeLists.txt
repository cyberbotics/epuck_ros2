cmake_minimum_required(VERSION 3.5)
project(epuck_ros2_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

add_executable(camera 
	src/camera.cpp
  src/pipuck_v4l2.c
  src/pipuck_jpeg.c
  src/pipuck_ov7670.c
  src/pipuck_image.c
)

target_include_directories(camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /opt/vc/include
  /opt/vc/include/interface/vcos/pthreads
  /opt/vc/include/interface/vmcs_host/linux
)

ament_target_dependencies(camera
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

target_link_libraries(camera 
  ${MMAL_LIBS}
)

install(TARGETS camera
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
