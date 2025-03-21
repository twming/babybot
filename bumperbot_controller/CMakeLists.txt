cmake_minimum_required(VERSION 3.8)
project(bumperbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# For C++ Code
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(simple_diffdrive_differential_IK_odom_msg_broadcast_controller src/simple_diffdrive_differential_IK_odom_msg_broadcast_controller.cpp)
ament_target_dependencies(simple_diffdrive_differential_IK_odom_msg_broadcast_controller rclcpp geometry_msgs std_msgs ${Eigen_LIBRARIES} sensor_msgs nav_msgs tf2 tf2_ros)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  simple_diffdrive_differential_IK_odom_msg_broadcast_controller
  DESTINATION lib/${PROJECT_NAME}
)

# For Python Script

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/simple_diffdrive_differential_IK_odom_msg_broadcast_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
