cmake_minimum_required(VERSION 3.8)
project(zva_sd0_beadando)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(ament_dependencies
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(gen_node src/gen.cpp)
ament_target_dependencies(gen_node ${ament_dependencies} )
add_executable(sum_node src/sum.cpp)
ament_target_dependencies(sum_node ${ament_dependencies} )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
 gen_node
 sum_node
DESTINATION lib/${PROJECT_NAME})

ament_package()