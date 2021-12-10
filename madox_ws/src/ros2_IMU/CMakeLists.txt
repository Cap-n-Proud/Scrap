cmake_minimum_required(VERSION 3.5)
project(ros2_IMU)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ros2_imu_node src/imu.cpp)
target_link_libraries(ros2_imu_node RTIMULib)


ament_target_dependencies(ros2_imu_node rclcpp std_msgs)


## Specify additional locations of header files
include_directories( include ${CMAKE_CURRENT_SOURCE_DIR} include ${catkin_INCLUDE_DIRS} /usr/local/include)

install(TARGETS
  ros2_imu_node
  DESTINATION lib/${PROJECT_NAME}

  )
ament_package()
