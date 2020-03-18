cmake_minimum_required(VERSION 3.5)
project(epuck_ros2_cpp)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(driver
  src/driver.cpp
  src/vl53l0x/tof.c
)
target_include_directories(driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  src/vl53l0x
)

ament_target_dependencies(driver
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  nav_msgs
)

install(TARGETS driver
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

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

  # `CONFIG_FILE` parameter hasn't been integrated into Debian upstream packages, but as soon as it is
  # we can run the uncrustify test as well:
  # https://github.com/ament/ament_lint/blob/091e47cb123d903926911bfdd6ca21672e5deae4/ament_cmake_uncrustify/cmake/ament_uncrustify.cmake#L56
  # find_package(ament_cmake_uncrustify REQUIRED)
  # ament_uncrustify(
  #   TESTNAME "uncrustify_${PROJECT_NAME}"
  #   ARGN ${PACKAGE_SOURCE_FILES}
  #   CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../uncrustify.cfg"
  #   MAX_LINE_LENGTH 128
  #   "${_output_path}"
  # )
  add_launch_test(
    "test/test_driver.py"
    TIMEOUT 60
  )
endif()

ament_package()
