cmake_minimum_required(VERSION 3.5)
project(robot_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Add include directories
include_directories(include)

# Generate URDF from Xacro
set(urdf_file ${PROJECT_SOURCE_DIR}/urdf/robot.urdf)
set(xacro_file ${PROJECT_SOURCE_DIR}/urdf/robot.xacro)
set(generate_urdf_cmd "${PYTHON_EXECUTABLE} ${xacro_EXECUTABLE} ${xacro_file} -o ${urdf_file}")
execute_process(
  COMMAND ${generate_urdf_cmd}
)

# Install URDF file
install(
  FILES ${urdf_file}
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Create executable for motor_controller node
add_executable(motor_controller
  src/motor_controller.cpp
)
ament_target_dependencies(motor_controller rclcpp)

# Create executable for imu node
add_executable(imu
  src/imu.cpp
)
ament_target_dependencies(imu rclcpp sensor_msgs)

# Create executable for odometer node
add_executable(odometer
  src/odometer.cpp
)
ament_target_dependencies(odometer rclcpp sensor_msgs)

# Create executable for lidar node
add_executable(lidar
  src/lidar.cpp
)
ament_target_dependencies(lidar rclcpp sensor_msgs)

# Create executable for robot_control node
add_executable(robot_control
  src/robot_control.cpp
)
ament_target_dependencies(robot_control rclcpp sensor_msgs urdf)

# Install executables
install(TARGETS
  motor_controller
  imu
  odometer
  lidar
  robot_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
