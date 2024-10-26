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
    LegIKController() : Node("leg_ik_controller"),
                        left_leg_move_group_interface_(std::make_shared<rclcpp::Node>(this->get_name()), "left_leg_group"),
                        right_leg_move_group_interface_(std::make_shared<rclcpp::Node>(this->get_name()), "right_leg_group")
    {
      // test_target_pose();
      test_named_target_pose("primed");
    }

  private:
    void test_named_target_pose(const std::string& target_pose_name) {
      left_leg_move_group_interface_.setNamedTarget(target_pose_name);
      right_leg_move_group_interface_.setNamedTarget(target_pose_name);
      RCLCPP_INFO(this->get_logger(), "Go go go!"); 

      std::thread left_leg_thread(&LegIKController::plan_and_execute, this, std::ref(left_leg_move_group_interface_), target_pose_name);
      std::thread right_leg_thread(&LegIKController::plan_and_execute, this, std::ref(right_leg_move_group_interface_), target_pose_name);

      left_leg_thread.join();
      right_leg_thread.join();

      // left_leg_move_group_interface_.asyncMove();
      // right_leg_move_group_interface_.asyncMove();

    //   moveit::planning_interface::MoveGroupInterface::Plan left_leg_plan;
    //   bool left_leg_success = (left_leg_move_group_interface_.plan(left_leg_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //   moveit::planning_interface::MoveGroupInterface::Plan right_leg_plan;
    //   bool right_leg_success = (right_leg_move_group_interface_.plan(right_leg_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //   if (left_leg_success && right_leg_success) {
    //     RCLCPP_INFO(this->get_logger(), "Yipeeeeee!"); 
    //     left_leg_move_group_interface_.asyncExecute(left_leg_plan);
    //     right_leg_move_group_interface_.asyncExecute(right_leg_plan);
    //   } else {
    //     RCLCPP_INFO(this->get_logger(), "Planning failed!"); 
    //   }
    }

    void plan_and_execute(moveit::planning_interface::MoveGroupInterface& move_group_interface, const std::string& target_pose_name) {
      move_group_interface.setNamedTarget(target_pose_name);
      move_group_interface.asyncMove();
      return;

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success) {
        RCLCPP_INFO(this->get_logger(), "Planning successful for %s. Executing plan...", move_group_interface.getName().c_str());
        move_group_interface.asyncMove();
        // if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        //   RCLCPP_INFO(this->get_logger(), "Execution successful for %s.", move_group_interface.getName().c_str());
        // } else {
        //   RCLCPP_ERROR(this->get_logger(), "Execution failed for %s.", move_group_interface.getName().c_str());
        // }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed for %s!", move_group_interface.getName().c_str());
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

    moveit::planning_interface::MoveGroupInterface left_leg_move_group_interface_;
    moveit::planning_interface::MoveGroupInterface right_leg_move_group_interface_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegIKController>());
  rclcpp::shutdown();
  return 0;
}
