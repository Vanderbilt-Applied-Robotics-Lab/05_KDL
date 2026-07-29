cmake_minimum_required(VERSION 3.8)
project(inv_kin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)

## INCLUDE(S)
include_directories(include)

# executables
add_executable(inv_kin_node src/inv_kin_node.cpp)

# dependencies
ament_target_dependencies(inv_kin_node rclcpp std_msgs kdl_parser sensor_msgs)

target_link_libraries(inv_kin_node ${KDL_PARSER_LIBRARIES})

install(TARGETS inv_kin_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
