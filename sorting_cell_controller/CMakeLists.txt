cmake_minimum_required(VERSION 3.8)
project(sorting_cell_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(fk_demo src/fk_demo.cpp)
target_include_directories(fk_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(fk_demo Eigen3::Eigen)

add_executable(fk_checker src/fk_checker.cpp)
target_include_directories(fk_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fk_checker rclcpp sensor_msgs geometry_msgs tf2_ros)
target_link_libraries(fk_checker Eigen3::Eigen)

add_executable(ik_test src/ik_test.cpp)
target_include_directories(ik_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ik_test Eigen3::Eigen)

add_executable(traj_test src/traj_test.cpp)
target_include_directories(traj_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(traj_test Eigen3::Eigen)

add_executable(cartesian_move_demo src/cartesian_move_demo.cpp)
target_include_directories(cartesian_move_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cartesian_move_demo rclcpp sensor_msgs)
target_link_libraries(cartesian_move_demo Eigen3::Eigen)

install(TARGETS fk_demo fk_checker ik_test traj_test cartesian_move_demo
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
