cmake_minimum_required(VERSION 3.5)
project(tracer_base)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tracer_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ugv_sdk REQUIRED)
find_package(nav_msgs REQUIRED)


include_directories(
  include
)

set(DEPENDENCIES
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "tracer_msgs"
  "sensor_msgs"
)

add_executable(tracer_base src/tracer_base_node.cpp src/tracer_messenger.cpp)
target_link_libraries(tracer_base /usr/local/lib/libugv_sdk.a)
ament_target_dependencies(tracer_base rclcpp std_msgs sensor_msgs tracer_msgs tf2 tf2_ros nav_msgs geometry_msgs)

install(
  TARGETS tracer_base
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
