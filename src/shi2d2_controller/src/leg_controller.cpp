#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "urdf/model.h"
#include "srdfdom/model.h"

// #include "shi2d2_left_leg_group_ikfast_plugin/shi2d2_left_leg_group_ik_fast_moveit_plugin.cpp"
// #include "shi2d2_left_leg_group_ikfast_plugin/ikfast.h"
#include "shi2d2_leg_group_ikfast_plugin/ikfast.h"
// #include "shi2d2_leg_group_ikfast_plugin/shi2d2_left_leg_group_ik_fast_moveit_plugin.cpp"

// using namespace std::chrono_literals;

class LegController : public rclcpp::Node
{
  public:
    LegController() : Node("leg_controller"), count_(0)
    {
      left_leg_joint_traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("left_leg_group_controller/joint_trajectory", 10);
      right_leg_joint_traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("right_leg_group_controller/joint_trajectory", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LegController::timer_callback, this));

      start_time_ = rclcpp::Time();
      clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", 10,
        [this](rosgraph_msgs::msg::Clock::SharedPtr msg) {
            clock_callback(msg);
        });
      
      left_leg_controller_joint_angles_["left_upper_hip_body_joint"] = 0.0;
      left_leg_controller_joint_angles_["left_lower_hip_left_upper_hip_joint"] = 0.0;
      left_leg_controller_joint_angles_["left_upper_leg_left_lower_hip_joint"] = 0.0;
      left_leg_controller_joint_angles_["left_lower_leg_left_upper_leg_joint"] = 0.0;
      left_leg_controller_joint_angles_["left_foot_left_lower_leg_joint"] = 0.0;

      right_leg_controller_joint_angles_["right_upper_hip_body_joint"] = 0.0;
      right_leg_controller_joint_angles_["right_lower_hip_right_upper_hip_joint"] = 0.0;
      right_leg_controller_joint_angles_["right_upper_leg_right_lower_hip_joint"] = 0.0;
      right_leg_controller_joint_angles_["right_lower_leg_right_upper_leg_joint"] = 0.0;
      right_leg_controller_joint_angles_["right_foot_right_lower_leg_joint"] = 0.0;

      /*
      // test_ikfast_class_ = std::make_shared<TestIKFastClass>();
      test_A_ = std::make_shared<A>();

      const std::string urdf_file_path = "../../shi2d2_moveit_config/config/shi2d2.urdf.xacro";
      const std::string srdf_file_path = "../../shi2d2_moveit_config/config/shi2d2.srdf";

      urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(urdf_file_path);
      if (!urdf_model)
      {
        throw std::runtime_error("Failed to load URDF model");
      }

      // Load SRDF model
      srdf::Model srdf_model;
      if (!srdf_model.initFile(*urdf_model, srdf_file_path))
      {
        throw std::runtime_error("Failed to load SRDF model");
      }

      // Create RobotModel
      moveit::core::RobotModel robot_model;
      robot_model.buildModel(*urdf_model, srdf_model);


      // bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
      //                 const std::string& group_name, const std::string& base_frame,
      //                 const std::vector<std::string>& tip_frames, double search_discretization) override;
      left_leg_ikfast_kinematics_plugin_ = std::make_shared<IKFastKinematicsPlugin>();
      left_leg_ikfast_kinematics_plugin_->initialize(shared_from_this(), &robot_model, "left_leg_group", "left_upper_hip_link", "left_foot_link", 0.01);

      */
    }

  private:
    void add_joint_position(trajectory_msgs::msg::JointTrajectory::SharedPtr traj, trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr point,
                            const std::string& joint_name, double joint_angle)
    {
      traj->joint_names.push_back(joint_name);
      point->positions.push_back(joint_angle);
    }

    void timer_callback()
    {
      // int a = 1;
      // int x = test_A_->get_id();
      // test_A_->double_id();
      // RCLCPP_INFO(this->get_logger(), "Test Class ID: %d", x);

      // int x = 3;
      // int y = test_triple_num(x);
      // RCLCPP_INFO(this->get_logger(), "Test Triple Num: %d", y);
      // int x = test_ikfast_class_->get_id();
      // RCLCPP_INFO(this->get_logger(), "Test Class ID: %d", x);
      // bool
      // getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
      //               std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
      //               const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;
                    
      // getPositionIK();

      return;

      auto message = trajectory_msgs::msg::JointTrajectory();
      auto point = trajectory_msgs::msg::JointTrajectoryPoint();
      auto left_leg_joint_traj = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
      auto left_leg_joint_traj_point = std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>();
      auto right_leg_joint_traj = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
      auto right_leg_joint_traj_point = std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>();

      double left_leg_joint_angle = M_PI/4.0 * cos(sim_time_elapsed_sec_ * 1.5 * M_PI) - M_PI/4.0;
      double right_leg_joint_angle = M_PI/4.0 * cos(sim_time_elapsed_sec_ * 1.5 * M_PI) - M_PI/4.0;

      left_leg_controller_joint_angles_["left_upper_leg_left_lower_hip_joint"] = -0.5 * left_leg_joint_angle;
      left_leg_controller_joint_angles_["left_lower_leg_left_upper_leg_joint"] = left_leg_joint_angle;
      left_leg_controller_joint_angles_["left_foot_left_lower_leg_joint"] = -0.5 * left_leg_joint_angle;

      right_leg_controller_joint_angles_["right_upper_leg_right_lower_hip_joint"] = 0.5 * right_leg_joint_angle;
      right_leg_controller_joint_angles_["right_lower_leg_right_upper_leg_joint"] = right_leg_joint_angle;
      right_leg_controller_joint_angles_["right_foot_right_lower_leg_joint"] = -0.5 * right_leg_joint_angle;

      // add_joint_position(joint_traj, joint_traj_point, "left_upper_hip_body_joint", 0.0);
      // add_joint_position(joint_traj, joint_traj_point, "left_lower_hip_left_upper_hip_joint", 0.0);
      // add_joint_position(joint_traj, joint_traj_point, "left_upper_leg_left_lower_hip_joint", -0.5 * left_leg_joint_angle);
      // add_joint_position(joint_traj, joint_traj_point, "left_lower_leg_left_upper_leg_joint", left_leg_joint_angle);
      // add_joint_position(joint_traj, joint_traj_point, "left_foot_left_lower_leg_joint", -0.5 * left_leg_joint_angle);
      
      // add_joint_position(joint_traj, joint_traj_point, "right_upper_hip_body_joint", 0.0);
      // add_joint_position(joint_traj, joint_traj_point, "right_lower_hip_right_upper_hip_joint", 0.0);
      // add_joint_position(joint_traj, joint_traj_point, "right_upper_leg_right_lower_hip_joint", 0.5 * right_leg_joint_angle);
      // add_joint_position(joint_traj, joint_traj_point, "right_lower_leg_right_upper_leg_joint", right_leg_joint_angle);
      // add_joint_position(joint_traj, joint_traj_point, "right_foot_right_lower_leg_joint", -0.5 * right_leg_joint_angle);

      std::map<std::string, double>::iterator left_leg_controller_joint_angles_iter = left_leg_controller_joint_angles_.begin();
      std::map<std::string, double>::iterator right_leg_controller_joint_angles_iter = right_leg_controller_joint_angles_.begin();

      // Iterate through the map and print the elements
      while (left_leg_controller_joint_angles_iter != left_leg_controller_joint_angles_.end()) {
        left_leg_joint_traj->joint_names.push_back(left_leg_controller_joint_angles_iter->first);
        left_leg_joint_traj_point->positions.push_back(left_leg_controller_joint_angles_iter->second);
        ++left_leg_controller_joint_angles_iter;
      }
      
      left_leg_joint_traj_point->time_from_start.nanosec = 0.01e9;
      left_leg_joint_traj->points.push_back(*left_leg_joint_traj_point);
      left_leg_joint_traj_publisher_->publish(*left_leg_joint_traj);

      while (right_leg_controller_joint_angles_iter != right_leg_controller_joint_angles_.end()) {
        right_leg_joint_traj->joint_names.push_back(right_leg_controller_joint_angles_iter->first);
        right_leg_joint_traj_point->positions.push_back(right_leg_controller_joint_angles_iter->second);
        ++right_leg_controller_joint_angles_iter;
      }
      
      right_leg_joint_traj_point->time_from_start.nanosec = 0.01e9;
      right_leg_joint_traj->points.push_back(*right_leg_joint_traj_point);
      right_leg_joint_traj_publisher_->publish(*right_leg_joint_traj);

      count_++;

      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f' and '%f", left_leg_joint_angle, right_leg_joint_angle);
    }

    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
      if (start_time_.nanoseconds() == 0) {
          start_time_ = msg->clock;
      }
      sim_time_ = msg->clock;
      
      if (start_time_.nanoseconds() != 0) {
        auto sim_time_elapsed_ = sim_time_ - start_time_;
        sim_time_elapsed_sec_ = sim_time_elapsed_.seconds();
        // RCLCPP_INFO(this->get_logger(), "Time since simulation start: %.2fs...", sim_time_elapsed_sec_);
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_leg_joint_traj_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_leg_joint_traj_publisher_;
    size_t count_;

    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscriber_;
    rclcpp::Time start_time_;
    rclcpp::Time sim_time_;
    double sim_time_elapsed_sec_;

    std::map<std::string, double> left_leg_controller_joint_angles_;
    std::map<std::string, double> right_leg_controller_joint_angles_;

    std::shared_ptr<IKFastKinematicsPlugin> left_leg_ikfast_kinematics_plugin_;
    std::shared_ptr<A> test_A_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegController>());
  rclcpp::shutdown();
  return 0;
}
