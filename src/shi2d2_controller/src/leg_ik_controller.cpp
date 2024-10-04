#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"
#include "geometry_msgs/msg/pose.hpp"

class LegIKController : public rclcpp::Node
{
  public:
    // LegIKController() : Node("leg_ik_controller"),
    //       robot_model_loader_(std::make_shared<rclcpp::Node>(this->get_name()), "shi2d2"),
    //       robot_model_(robot_model_loader_.getModel()),
    //       robot_state_(std::make_shared<moveit::core::RobotState>(robot_model_)),
    //       planning_group_("left_leg_group")
    // {   
    //     joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
    //     this->sampleSolveIK();
    // }

    // void sampleSolveIK()
    // {
    //     moveit::planning_interface::MoveGroupInterface move_group_interface(std::make_shared<rclcpp::Node>(this->get_name()), planning_group_);
        
    //     // get current pose, increase its z coordinate by 5 cm and solve IK to see if the new pose is reachable
    //     auto current_pose = move_group_interface.getCurrentPose();
    //     geometry_msgs::msg::Pose target_pose;
    //     target_pose.position.x = current_pose.pose.position.x;
    //     target_pose.position.y = current_pose.pose.position.y;
    //     target_pose.position.z = current_pose.pose.position.z + 0.05; // Fixed to directly modify the z coordinate
        
    //     bool found_ik = robot_state_->setFromIK(joint_model_group_, target_pose);
    //     if (found_ik)
    //     {
    //         RCLCPP_INFO(this->get_logger(), "IK solution found.");
    //     }
    //     else
    //     {
    //         RCLCPP_INFO(this->get_logger(), "IK solution not found.");
    //     }
    // }

    LegIKController() : Node("leg_ik_controller"),
                        left_leg_move_group_interface_(std::make_shared<rclcpp::Node>(this->get_name()), "left_leg_group")
    {
      // moveit::planning_interface::MoveGroupInterface left_leg_move_group_interface_(std::make_shared<rclcpp::Node>(this->get_name()), "left_leg_group");

      // test_target_pose();
      test_named_target_pose("primed");
    }

  private:
    void test_named_target_pose(const std::string& target_pose_name) {
      left_leg_move_group_interface_.setNamedTarget(target_pose_name);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = (left_leg_move_group_interface_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success) {
        RCLCPP_INFO(this->get_logger(), "Yipeeeeee!"); 
        left_leg_move_group_interface_.execute(plan);
      } else {
        RCLCPP_INFO(this->get_logger(), "Planning failed!"); 
      }
    }

    void test_target_pose() {
      auto const current_pose = left_leg_move_group_interface_.getCurrentPose();
      auto const target_pose = [current_pose]{
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = current_pose.pose.orientation.w;
        msg.position.x = current_pose.pose.position.x;
        msg.position.y = current_pose.pose.position.y;
        msg.position.z = current_pose.pose.position.z + 0.005;
        return msg;
      }();
      left_leg_move_group_interface_.setPoseTarget(target_pose);

      auto const [success, plan] = [this]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(left_leg_move_group_interface_.plan(msg));
        return std::make_pair(ok, msg);
      }();

      if (success) {
        left_leg_move_group_interface_.execute(plan);
      } else {
        RCLCPP_INFO(this->get_logger(), "Planning failed!"); 
      }
    }

    // using moveit::planning_interface::MoveGroupInterface;
    moveit::planning_interface::MoveGroupInterface left_leg_move_group_interface_;
    // moveit::planning_interface::MoveGroupInterface left_leg_move_group_interface;
    // moveit::planning_interface::MoveGroupInterface left_leg_move_group_interface(move_group_node, PLANNING_GROUP);

    // robot_model_loader::RobotModelLoader robot_model_loader_;
    // moveit::core::RobotModelPtr robot_model_;
    // moveit::core::RobotStatePtr robot_state_;
    // std::string planning_group_;
    // const moveit::core::JointModelGroup* joint_model_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegIKController>());
  rclcpp::shutdown();
  return 0;
}
