# XentrinoBot-ROS2
.
├── CMakeLists.txt
├── launch
│   ├── robot_description.launch.py
│   ├── motor_controller.launch.py
│   ├── imu.launch.py
│   ├── odometer.launch.py
│   ├── lidar.launch.py
│   └── robot_control.launch.py
├── package.xml
├── src
│   ├── motor_controller.cpp
│   ├── imu.cpp
│   ├── odometer.cpp
│   ├── lidar.cpp
│   └── robot_control.cpp
└── include
    ├── motor_controller.hpp
    ├── imu.hpp
    ├── odometer.hpp
    ├── lidar.hpp
    └── robot_control.hpp


Here's a brief explanation of what each file and directory does:

CMakeLists.txt: This file is used by CMake to build the ROS2 package. It specifies the dependencies and includes the necessary files.
launch: This directory contains launch files for starting the various nodes in the system.
robot_description.launch.py: This file launches the robot's URDF description.
motor_controller.launch.py: This file launches the node that controls the robot's motors.
imu.launch.py: This file launches the node that reads data from the MPU6050.
odometer.launch.py: This file launches the node that reads data from the odometer.
lidar.launch.py: This file launches the node that reads data from the Lidar.
robot_control.launch.py: This file launches the main node that integrates data from the other nodes and controls the robot.
package.xml: This file contains information about the ROS2 package, such as the package name and version.
src: This directory contains the source code for the various nodes.
motor_controller.cpp: This file contains the code for the node that controls the robot's motors.
imu.cpp: This file contains the code for the node that reads data from the MPU6050.
odometer.cpp: This file contains the code for the node that reads data from the odometer.
lidar.cpp: This file contains the code for the node that reads data from the Lidar.
robot_control.cpp: This file contains the code for the main node that integrates data from the other nodes and controls the robot.
include: This directory contains header files for the source code



.
├── CMakeLists.txt

├── launch

│   ├── robot_description.launch.py

│   ├── motor_controller.launch.py

│   ├── imu.launch.py

│   ├── odometer.launch.py

│   ├── lidar.launch.py

│   └── robot_control.launch.py

├── package.xml

├── src

│   ├── motor_controller.cpp

│   ├── imu.cpp

│   ├── odometer.cpp

│   ├── lidar.cpp

│   └── robot_control.cpp

└── include

    ├── motor_controller.hpp
    
    ├── imu.hpp
    
    ├── odometer.hpp
    
    ├── lidar.hpp
    
    └── robot_control.hpp
    
