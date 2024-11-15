cmake_minimum_required(VERSION 3.8)
project(rb1_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)


find_package(rb1_interfaces REQUIRED)
find_package(rb1_utils REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
include_directories( include )

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  behaviortree_cpp
  geometry_msgs
  tf2_ros
  nav2_msgs
  tf2_geometry_msgs

  rclcpp_action

  rb1_utils
  rb1_interfaces
  behaviortree_ros2
)

set(plugins_bt
  src/plugins_bt/client_find_object.cpp
  src/plugins_bt/client_init_localization.cpp
  src/plugins_bt/client_approach_shelf.cpp
  src/plugins_bt/client_nav.cpp

  src/plugins_bt/publish_transform.cpp
  src/plugins_bt/handler_platform.cpp
  src/plugins_bt/change_footprint.cpp
  src/plugins_bt/turn_robot.cpp
  src/plugins_bt/nav_poses.cpp
  src/plugins_bt/wait_for_goal_nav.cpp
  src/plugins_bt/check_approach.cpp
  
)
# servers
add_executable( main_server_find_object src/servers/server_find_object.cpp src/main_server_find_object.cpp)
ament_target_dependencies(main_server_find_object ${dependencies})

# servers
add_executable( main_server_approach_shelf src/servers/server_approach_shelf.cpp src/main_server_approach_shelf.cpp)
ament_target_dependencies(main_server_approach_shelf ${dependencies})

# servers
add_executable( main_server_init_localization src/servers/server_init_localization.cpp src/main_server_init_localization.cpp)
ament_target_dependencies(main_server_init_localization ${dependencies})


add_executable(main_autonomy src/main_autonomy.cpp src/autonomy.cpp ${plugins_bt})
ament_target_dependencies(main_autonomy ${dependencies})


install(TARGETS
  main_server_find_object
  main_server_approach_shelf
  main_server_init_localization

  main_autonomy

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  bt
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
