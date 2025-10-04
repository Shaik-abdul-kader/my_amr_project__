cmake_minimum_required(VERSION 3.10)
project(my_amr_project__)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(turtlebot3_gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Declaring the C++ executable
add_executable(goalpose src/goalpose.cpp)
add_executable(homepose src/goalpoints.cpp)

ament_target_dependencies(goalpose
  rclcpp rclcpp_action nav2_msgs geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs
)

ament_target_dependencies(homepose
  rclcpp
  std_msgs
  geometry_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectDetect.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

install(TARGETS
  goalpose
  homepose
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds config maps
  DESTINATION share/${PROJECT_NAME}
)

# Install the executable
install(PROGRAMS
    scripts/detection.py
    scripts/path_goal.py
    scripts/auto_move.py
    DESTINATION lib/${PROJECT_NAME}
)

#ament_python_install_package(${PROJECT_NAME})

# Export package dependencies for downstream packages
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  action_msgs
  rosidl_default_runtime
)

# Optional testing section
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
