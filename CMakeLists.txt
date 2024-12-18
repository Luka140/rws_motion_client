cmake_minimum_required(VERSION 3.8)
project(rws_motion_client)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(abb_robot_msgs REQUIRED)
find_package(stamped_std_msgs REQUIRED)
find_package(data_gathering_msgs REQUIRED)
find_package(ferrobotics_acf REQUIRED)

# Add the executable
add_executable(motion_client src/rws_motion_client.cpp)

# Link against ROS 2 libraries
ament_target_dependencies(motion_client
    rclcpp
    std_srvs
    abb_robot_msgs
    stamped_std_msgs
    data_gathering_msgs
    ferrobotics_acf
)

# Install the executable
install(TARGETS motion_client
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
