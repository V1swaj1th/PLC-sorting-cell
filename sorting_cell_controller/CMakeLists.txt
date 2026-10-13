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
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

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

add_executable(sort_plan_test src/sort_plan_test.cpp)
target_include_directories(sort_plan_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(sort_plan_test Eigen3::Eigen)

add_executable(sorting_sequencer src/sorting_sequencer.cpp)
target_include_directories(sorting_sequencer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sorting_sequencer rclcpp sensor_msgs geometry_msgs std_msgs visualization_msgs)
target_link_libraries(sorting_sequencer Eigen3::Eigen ${YAML_CPP_TARGET})

add_executable(plc_link_test src/plc_link_test.cpp)
target_include_directories(plc_link_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MODBUS_INCLUDE_DIRS})
target_link_libraries(plc_link_test ${MODBUS_LIBRARIES} pthread)

add_executable(plc_bridge src/plc_bridge.cpp)
target_include_directories(plc_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MODBUS_INCLUDE_DIRS})
ament_target_dependencies(plc_bridge rclcpp std_msgs)
target_link_libraries(plc_bridge ${MODBUS_LIBRARIES})

add_executable(validation_report src/validation_report.cpp)
target_include_directories(validation_report PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(validation_report Eigen3::Eigen)

add_executable(teach_test src/teach_test.cpp)
target_include_directories(teach_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(teach_test Eigen3::Eigen ${YAML_CPP_TARGET})

add_executable(teach_pendant src/teach_pendant.cpp)
target_include_directories(teach_pendant PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(teach_pendant rclcpp sensor_msgs geometry_msgs std_msgs visualization_msgs)
target_link_libraries(teach_pendant Eigen3::Eigen ${YAML_CPP_TARGET})

add_executable(scene_test src/scene_test.cpp)
target_include_directories(scene_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(scene_test Eigen3::Eigen ${YAML_CPP_TARGET})

add_executable(scene_check src/scene_check.cpp)
target_include_directories(scene_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(scene_check Eigen3::Eigen ${YAML_CPP_TARGET})

add_executable(scene_server src/scene_server.cpp)
target_include_directories(scene_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(scene_server rclcpp std_msgs)
target_link_libraries(scene_server Eigen3::Eigen ${YAML_CPP_TARGET})

install(TARGETS fk_demo fk_checker ik_test traj_test cartesian_move_demo sort_plan_test sorting_sequencer plc_link_test plc_bridge validation_report teach_test teach_pendant scene_test scene_check scene_server
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY programs/ DESTINATION share/${PROJECT_NAME}/programs)

ament_package()
