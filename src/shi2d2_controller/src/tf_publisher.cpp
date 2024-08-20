#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class TFPublisher : public rclcpp::Node
{
  public:
    TFPublisher() : Node("tf_publisher")
    {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/robot_pose", 10, std::bind(&TFPublisher::receive_pose_callback, this, std::placeholders::_1)
      );
    }

  private:
    void receive_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "base_link";
      t.child_frame_id = "head_link";
      t.transform.translation.x = msg->pose.position.x;
      t.transform.translation.y = msg->pose.position.y;
      t.transform.translation.z = msg->pose.position.z;
      t.transform.rotation = msg->pose.orientation;

      tf_broadcaster_->sendTransform(t);

      RCLCPP_INFO(this->get_logger(), "Publishing: '(%f, %f, %f)'", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFPublisher>());
  rclcpp::shutdown();
  return 0;
}
