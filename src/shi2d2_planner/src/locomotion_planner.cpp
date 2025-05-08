#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include "shi2d2_planner/shi2d2_planner.hpp"

using namespace std::chrono_literals;

class LocomotionPlanner : public rclcpp::Node
{
public:
  LocomotionPlanner() : Node("locomotion_planner")
  {
    // Publisher
    publisher_ = this->create_publisher<std_msgs::msg::Int8>("locomotion_commands", PLANNER_LOOP_PERIOD_MS);

    // Subscribers
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("/shi2d2/imu", PLANNER_LOOP_PERIOD_MS,
      std::bind(&LocomotionPlanner::imu_callback, this, std::placeholders::_1));

    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/shi2d2/odometry", PLANNER_LOOP_PERIOD_MS,
      std::bind(&LocomotionPlanner::odometry_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(PLANNER_LOOP_PERIOD_MS * 1ms, std::bind(&LocomotionPlanner::timer_callback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  ~LocomotionPlanner()
  {
    return;
  }

private:
  void quaternion_to_euler(double &x, double &y, double &z, double &w, double &roll, double &pitch, double &yaw) {
    tf2::Quaternion tf_q(x, y, z, w);
    tf2::Matrix3x3 m(tf_q);
    m.getRPY(roll, pitch, yaw);
  }

  void get_robot_com()
  {

  }

  void get_robot_state()
  {
    // Get the robot state from the IMU and odometry data
    body_state_.x = body_odometry_.pose.pose.position.x;
    body_state_.y = body_odometry_.pose.pose.position.y;
    body_state_.z = body_odometry_.pose.pose.position.z;

    quaternion_to_euler(body_imu_data_.orientation.x, body_imu_data_.orientation.y, body_imu_data_.orientation.z, body_imu_data_.orientation.w,
                        body_state_.rx, body_state_.ry, body_state_.rz);

    body_state_.vx = body_odometry_.twist.twist.linear.x;
    body_state_.vy = body_odometry_.twist.twist.linear.y;
    body_state_.vz = body_odometry_.twist.twist.linear.z;

    body_state_.wx = body_imu_data_.angular_velocity.x;
    body_state_.wy = body_imu_data_.angular_velocity.y;
    body_state_.wz = body_imu_data_.angular_velocity.z;

    body_state_.ax = body_imu_data_.linear_acceleration.x;
    body_state_.ay = body_imu_data_.linear_acceleration.y;
    body_state_.az = body_imu_data_.linear_acceleration.z;

    body_state_.alx = 0.0; // TODO: is there a way to get angular acceleration if we need it?
    body_state_.aly = 0.0;
    body_state_.alz = 0.0;
    
    std::cout << std::fixed << std::setprecision(3)
          << "Body State: X: [" 
          << body_state_.x << ", " << body_state_.y << ", " << body_state_.z << ", "
          << body_state_.rx << ", " << body_state_.ry << ", " << body_state_.rz << "] XD: ["
          << body_state_.vx << ", " << body_state_.vy << ", " << body_state_.vz << ", "
          << body_state_.wx << ", " << body_state_.wy << ", " << body_state_.wz << "] XDD:["
          << body_state_.ax << ", " << body_state_.ay << ", " << body_state_.az << ", "
          << body_state_.alx << ", " << body_state_.aly << ", " << body_state_.alz << "]" << std::endl;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    body_imu_data_ = *msg;
    // RCLCPP_INFO(this->get_logger(), "IMU Acceleration: [%.2f, %.2f, %.2f, %.2f]",
    //             msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    body_odometry_ = *msg;
    // RCLCPP_INFO(this->get_logger(), "Odometry Position: [%.2f, %.2f, %.2f]",
    //             msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  }

  void timer_callback()
  {
    // Process TF data
    try
    {
      auto transform_stamped = tf_buffer_->lookupTransform("world", "com_link", tf2::TimePointZero);
      auto left_foot_transform_stamped = tf_buffer_->lookupTransform("com_link", "left_foot_link", tf2::TimePointZero);
      auto right_foot_transform_stamped = tf_buffer_->lookupTransform("com_link", "right_foot_link", tf2::TimePointZero);
      body_transform_ = transform_stamped.transform;
      // RCLCPP_INFO(this->get_logger(), "Positions: B: [%.2f, %.2f, %.2f] L: [%.2f, %.2f, %.2f] R: [%.2f, %.2f, %.2f]",
      //             body_transform_.translation.x,
      //             body_transform_.translation.y,
      //             body_transform_.translation.z,
      //             left_foot_transform_stamped.transform.translation.x,
      //             left_foot_transform_stamped.transform.translation.y,
      //             left_foot_transform_stamped.transform.translation.z,
      //             right_foot_transform_stamped.transform.translation.x,
      //             right_foot_transform_stamped.transform.translation.y,
      //             right_foot_transform_stamped.transform.translation.z);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
    get_robot_state();

    // Example publishing logic (can be customized)
    auto message = std_msgs::msg::Int8();
    message.data = 0; // Example command
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  geometry_msgs::msg::Transform body_transform_;
  nav_msgs::msg::Odometry body_odometry_;
  sensor_msgs::msg::Imu body_imu_data_;
  geometry_msgs::msg::Pose body_pose_;

  BodyState body_state_; 
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocomotionPlanner>());
  rclcpp::shutdown();
  return 0;
}