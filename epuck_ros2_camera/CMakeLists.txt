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
  add_compile_options(-Wall)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)

if(NOT AVOID_EPUCK_CAMERA_BUILD)
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
    src/pipuck_mmal.c
    src/pipuck_ov7670.c
  )

  target_include_directories(camera PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    /opt/vc/include
    /opt/vc/include/interface/vcos/pthreads
    /opt/vc/include/interface/vmcs_host/linux
  )

  ament_target_dependencies(camera
    rclcpp
    sensor_msgs
    camera_info_manager
  )

  target_link_libraries(camera
    ${MMAL_LIBS}
  )

  install(TARGETS camera
    EXPORT export_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY camera_info/
    DESTINATION share/epuck_ros2_camera/camera_info
  )
endif()

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  set(PACKAGE_CPP_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  )

  ament_cppcheck(
    ARGN ${PACKAGE_CPP_FILES}
  )
  ament_cpplint(
    ARGN ${PACKAGE_CPP_FILES}
    MAX_LINE_LENGTH 128
  )
  ament_copyright(
    ARGN ${PACKAGE_CPP_FILES}
  )
  ament_lint_cmake()
  ament_flake8()
  ament_pep257()
  ament_xmllint()
  ament_clang_format(
    ARGN ${PACKAGE_CPP_FILES}
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../.clang-format"
  )
endif()

ament_package()
