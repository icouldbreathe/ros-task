cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

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

add_executable(mysqlnode src/mysql_node.cpp)
target_link_libraries(mysqlnode mysqlcppconn)
ament_target_dependencies(mysqlnode rclcpp std_msgs geometry_msgs)

install(TARGETS
  mysqlnode
  DESTINATION lib/${PROJECT_NAME})

ament_package()