cmake_minimum_required(VERSION 3.8)
project(servo_core CXX)
# Plain C++ library, no ROS code. It builds with colcon (ament_cmake) or with plain CMake.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake QUIET)
find_package(Eigen3 REQUIRED NO_MODULE)

# Firmware: what would run on a drive's microcontroller. float, no heap, no exceptions, no RTTI.
add_library(servo_firmware STATIC src/pid.cpp src/servo_axis.cpp src/drive.cpp)
target_include_directories(servo_firmware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(servo_firmware PRIVATE
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wdouble-promotion -fno-exceptions -fno-rtti)
set_target_properties(servo_firmware PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Simulation: plant and arm model (double precision, Eigen), header-only.
add_library(servo_sim INTERFACE)
target_include_directories(servo_sim INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(servo_sim INTERFACE servo_firmware Eigen3::Eigen)

if(ament_cmake_FOUND)
  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS servo_firmware servo_sim EXPORT export_servo_core
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  ament_export_targets(export_servo_core HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3)
  find_package(ament_cmake_gtest QUIET)
  if(BUILD_TESTING AND NOT ament_cmake_gtest_FOUND)
    message(WARNING "ament_cmake_gtest not found: servo_core tests are not built (apt install ros-jazzy-ament-cmake-gtest)")
  endif()
  if(BUILD_TESTING AND ament_cmake_gtest_FOUND)
    ament_add_gtest(servo_test test/test_servo.cpp TIMEOUT 120)
    target_link_libraries(servo_test servo_sim)
    add_test(NAME firmware_no_runtime_symbols
      COMMAND ${CMAKE_COMMAND} -DLIB=$<TARGET_FILE:servo_firmware> -P ${CMAKE_CURRENT_SOURCE_DIR}/test/check_symbols.cmake)
  endif()
  ament_package()
else()
  # stand-alone: cmake -DGTEST_SOURCE_DIR=/path/to/googletest
  enable_testing()
  if(GTEST_SOURCE_DIR)
    add_subdirectory(${GTEST_SOURCE_DIR} googletest EXCLUDE_FROM_ALL)
  else()
    find_package(GTest REQUIRED)
  endif()
  add_executable(servo_test test/test_servo.cpp)
  target_link_libraries(servo_test servo_sim GTest::gtest_main)
  add_test(NAME servo_test COMMAND servo_test)
  add_test(NAME firmware_no_runtime_symbols
    COMMAND ${CMAKE_COMMAND} -DLIB=$<TARGET_FILE:servo_firmware> -P ${CMAKE_CURRENT_SOURCE_DIR}/test/check_symbols.cmake)
endif()
