#include <moveit/move_group_interface/move_group_interface.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_moveit_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  auto arm_group = moveit::planning_interface::MoveGroupInterface(node, "arm");
  arm_group.setMaxVelocityScalingFactor(1.0);
  arm_group.setMaxAccelerationScalingFactor(1.0);

  auto gripper_group =
      moveit::planning_interface::MoveGroupInterface(node, "gripper");
  gripper_group.setMaxVelocityScalingFactor(1.0);
  gripper_group.setMaxAccelerationScalingFactor(1.0);

  // Name goal
  arm_group.setStartStateToCurrentState();
  arm_group.setNamedTarget("pose1");

  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  bool success1 =
      (arm_group.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(node->get_logger(), "Planning to named target pose1 %s",
              success1 ? "SUCCEEDED" : "FAILED");
  if (success1) {
    arm_group.execute(plan1);
  }

  gripper_group.setStartStateToCurrentState();
  gripper_group.setNamedTarget("gripper_half_closed");
  moveit::planning_interface::MoveGroupInterface::Plan plan_gripper;
  bool success_gripper = (gripper_group.plan(plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(node->get_logger(),
              "Planning to named target gripper_half_closed %s",
              success_gripper ? "SUCCEEDED" : "FAILED");
  if (success_gripper) {
    gripper_group.execute(plan_gripper);
  }

  arm_group.setStartStateToCurrentState();
  arm_group.setNamedTarget("pose2");

  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  bool success2 =
      (arm_group.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(node->get_logger(), "Planning to named target pose2 %s ",
              success2 ? "SUCCEEDED" : "FAILED");
  if (success2) {
    arm_group.execute(plan2);
  }

  gripper_group.setStartStateToCurrentState();
  gripper_group.setNamedTarget("gripper_open");
  moveit::planning_interface::MoveGroupInterface::Plan plan_gripper_open;
  bool success_gripper_open = (gripper_group.plan(plan_gripper_open) ==
                               moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(node->get_logger(), "Planning to named target gripper_open %s",
              success_gripper_open ? "SUCCEEDED" : "FAILED");
  if (success_gripper_open) {
    gripper_group.execute(plan_gripper_open);
  }

  arm_group.setStartStateToCurrentState();
  arm_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan plan3;
  bool success3 =
      (arm_group.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(node->get_logger(), "Planning to named target home %s ",
              success3 ? "SUCCEEDED" : "FAILED");
  if (success3) {
    arm_group.execute(plan3);
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}