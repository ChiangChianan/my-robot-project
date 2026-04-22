#include <moveit/move_group_interface/move_group_interface.h>

#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <my_robot_interfaces_pkg/msg/pose_command.hpp>
#include <rclcpp/rclcpp.hpp>

class MoveitCommander {
 public:
  explicit MoveitCommander(std::shared_ptr<rclcpp::Node> node) {
    node_ = node;
    arm_group_ = std::make_shared<MoveGroupInterface>(node_, "arm");
    arm_group_->setMaxVelocityScalingFactor(1.0);
    arm_group_->setMaxAccelerationScalingFactor(1.0);
    gripper_group_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
    gripper_group_->setMaxAccelerationScalingFactor(1.0);
    gripper_group_->setMaxVelocityScalingFactor(1.0);

    open_gripper_sub_ =
        node_->create_subscription<example_interfaces::msg::Bool>(
            "open_gripper", 10,
            std::bind(&MoveitCommander::OpenGripperCallback, this,
                      std::placeholders::_1));

    joint_cmd_sub_ =
        node_->create_subscription<example_interfaces::msg::Float64MultiArray>(
            "joint_cmd", 10,
            std::bind(&MoveitCommander::JointCmdCallback, this,
                      std::placeholders::_1));
    pose_cmd_sub_ =
        node_->create_subscription<my_robot_interfaces_pkg::msg::PoseCommand>(
            "pose_cmd", 10,
            std::bind(&MoveitCommander::PoseCmdCallback, this,
                      std::placeholders::_1));
  }

  void GoToNamedTarget(const std::string& name) {
    arm_group_->setStartStateToCurrentState();
    arm_group_->setNamedTarget(name);
    PlanAndExecute(arm_group_);
  }

  void GoToJointTarget(const std::vector<double>& joints) {
    arm_group_->setStartStateToCurrentState();
    arm_group_->setJointValueTarget(joints);
    PlanAndExecute(arm_group_);
  }

  void GoToPoseTarget(double x, double y, double z, double roll, double pitch,
                      double yaw, bool cartesian_path = false) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);  // Roll, Pitch, Yaw
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;

    target_pose.pose.orientation.x = q.x();
    target_pose.pose.orientation.y = q.y();
    target_pose.pose.orientation.z = q.z();
    target_pose.pose.orientation.w = q.w();

    if (!cartesian_path) {
      arm_group_->setPoseTarget(target_pose);
      PlanAndExecute(arm_group_);
    } else {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(target_pose.pose);

      moveit_msgs::msg::RobotTrajectory trajectory;
      double fraction =
          arm_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
      if (fraction >= 1.0 - 1e-6) {
        arm_group_->execute(trajectory);
        RCLCPP_INFO(node_->get_logger(),
                    "Cartesian path computed successfully. Executing...");
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "Could not compute a complete Cartesian path. Fraction: %f",
                    fraction);
      }
    }
  }

  void OpenGripper() {
    gripper_group_->setStartStateToCurrentState();
    gripper_group_->setNamedTarget("gripper_open");
    PlanAndExecute(gripper_group_);
  }

  void CloseGripper() {
    gripper_group_->setStartStateToCurrentState();
    gripper_group_->setNamedTarget("gripper_closed");
    PlanAndExecute(gripper_group_);
  }

 private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

  void PlanAndExecute(const std::shared_ptr<MoveGroupInterface>& interface) {
    MoveGroupInterface::Plan plan;
    bool success =
        (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node_->get_logger(), "Planning to target pose %s",
                success ? "SUCCEEDED" : "FAILED");
    if (success) {
      interface->execute(plan);
    }
  }

  void OpenGripperCallback(const example_interfaces::msg::Bool& msg) {
    if (msg.data) {
      OpenGripper();
    } else {
      CloseGripper();
    }
  }

  void JointCmdCallback(const example_interfaces::msg::Float64MultiArray& msg) {
    auto joints = msg.data;
    if (joints.size() == 6) {
      GoToJointTarget(joints);
    }
  }
  void PoseCmdCallback(const my_robot_interfaces_pkg::msg::PoseCommand& msg) {
    GoToPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw,
                   msg.cartesian_path);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      gripper_group_;

  rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr
      open_gripper_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr
      joint_cmd_sub_;
  rclcpp::Subscription<my_robot_interfaces_pkg::msg::PoseCommand>::SharedPtr
      pose_cmd_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("moveit_commander");
  auto commander = MoveitCommander(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}