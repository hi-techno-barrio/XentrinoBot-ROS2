
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class RobotControl : public rclcpp::Node
{
public:
  RobotControl()
  : Node("robot_control")
  {
    // Create subscriber for joint state data
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      10,
      std::bind(&RobotControl::jointStateCallback, this, std::placeholders::_1)
    );
    
    // Create subscriber for IMU data
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu",
      10,
      std::bind(&RobotControl::imuCallback, this, std::placeholders::_1)
    );
    
    // Create subscriber for odometry data
    odom_sub_ = this->create_subscription<sensor_msgs::msg::Odometry>(
      "odom",
      10,
      std::bind(&RobotControl::odomCallback, this, std::placeholders::_1)
    );
    
    // Create subscriber for LaserScan data
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      10,
      std::bind(&RobotControl::laserCallback, this, std::placeholders::_1)
    );
    
    // Initialize robot control
    // ...
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  

  
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Update robot control #include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class RobotControl : public rclcpp::Node
{
public:
  RobotControl()
  : Node("robot_control")
  {
    // Create subscriber for joystick data
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy",
      10,
      std::bind(&RobotControl::joyCallback, this, std::placeholders::_1)
    );
    
    // Create subscriber for IMU data
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu",
      10,
      std::bind(&RobotControl::imuCallback, this, std::placeholders::_1)
    );
    
    // Create subscriber for odometry data
    odom_sub_ = this->create_subscription<sensor_msgs::msg::Odometry>(
      "odom",
      10,
      std::bind(&RobotControl::odomCallback, this, std::placeholders::_1)
    );
    
    // Create subscriber for LaserScan data
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      10,
      std::bind(&RobotControl::laserCallback, this, std::placeholders::_1)
    );
    
    // Create publisher for velocity commands
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Initialize robot control
    vel_msg_.linear.x = 0.0;
    vel_msg_.linear.y = 0.0;
    vel_msg_.linear.z = 0.0;
    vel_msg_.angular.x = 0.0;
    vel_msg_.angular.y = 0.0;
    vel_msg_.angular.z = 0.0;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  
  geometry_msgs::msg::Twist vel_msg_;

 void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Example control logic: Move the robot arm to a certain position
  constexpr size_t JOINT_ARM_INDEX = 2; // Index of the joint controlling the arm
  constexpr float TARGET_ARM_POSITION = 1.0; // Target position for the arm joint in radians
  constexpr float JOINT_SPEED = 0.1; // Joint speed in radians per second

  if (msg->position.size() > JOINT_ARM_INDEX) {
    float current_position = msg->position[JOINT_ARM_INDEX];
    if (std::abs(current_position - TARGET_ARM_POSITION) > 0.1) {
      // Move the arm joint towards the target position
      float direction = (TARGET_ARM_POSITION > current_position) ? 1.0 : -1.0;
      float delta_position = direction * JOINT_SPEED * 0.1; // 0.1 is the time step in seconds
      vel_msg_.position[JOINT_ARM_INDEX] = current_position + delta_position;
      // Publish joint position command
      vel_pub_->publish(vel_msg_);
    }
  }
}

  
void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Example control logic: Stop the robot if it tilts too much
  constexpr float tilt_threshold = 0.5; // radians
  if (std::abs(msg->linear_acceleration.x) > tilt_threshold ||
      std::abs(msg->linear_acceleration.y) > tilt_threshold) {
    stopRobot();
  }
}


void odomCallback(const sensor_msgs::msg::Odometry::SharedPtr msg)
{
  // Example control logic: Move the robot to a random position
  if (!moving_) {
    target_position_ = 10.0 * static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) - 5.0; // Random position within [-5, 5] meters
  }
  current_position_ = msg->pose.pose.position.x;
}


  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Example control logic: Stop the robot if an obstacle is
// detected within a certain distance threshold
constexpr float obstacle_distance_threshold = 0.5; // meters
for (float range : msg->ranges) {
  if (range < obstacle_distance_threshold) {
    stopRobot();
    break;
  }
}
}

// Example function for starting the robot
void startRobot() {
vel_msg_.linear.x = 0.1; // Move forward at 0.1 m/s
vel_pub_->publish(vel_msg_);
moving_ = true;
}

// Example function for stopping the robot
void stopRobot() {
vel_msg_.linear.x = 0.0;
vel_msg_.angular.z = 0.0;
vel_pub_->publish(vel_msg_);
moving_ = false;
}

// Example function for controlling the robot based on sensor data
void controlRobot() {
// Check if there is an obstacle detected
if (obstacle_detected_) {
stopRobot();
obstacle_detected_ = false;
}// Check if the robot is not moving and start it
if (!moving_) {
  startRobot();
}

// Check if the robot has reached the target position
if (std::abs(current_position_ - target_position_) < 0.1) {
  stopRobot();
}

// Move the robot towards the target position
if (moving_) {
  if (target_position_ > current_position_) {
    // Move the robot forward
    vel_msg_.linear.x = 0.1; // Move forward at 0.1 m/s
  } else {
    // Move the robot backward
    vel_msg_.linear.x = -0.1; // Move backward at 0.1 m/s
  }
  vel_pub_->publish(vel_msg_);
}
}

rclcpp::TimerBase::SharedPtr control_timer_;
float current_position_ = 0.0;
float target_position_ = 1.0;
bool moving_ = false;
bool obstacle_detected_ = false;
};

int main(int argc, char **argv)
{
rclcpp::init(argc, argv);
auto node = std::make_shared<RobotControl>();
node->control_timer_ = node->create_wall_timer(std::chrono::milliseconds(10), std::bind(&RobotControl::controlRobot, node));
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}
