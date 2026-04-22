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

  // // Named goal
  // arm_group.setStartStateToCurrentState();
  // arm_group.setNamedTarget("pose1");

  // moveit::planning_interface::MoveGroupInterface::Plan plan1;
  // bool success1 =
  //     (arm_group.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(node->get_logger(), "Planning to named target pose1 %s",
  //             success1 ? "SUCCEEDED" : "FAILED");
  // if (success1) {
  //   arm_group.execute(plan1);
  // }

  // gripper_group.setStartStateToCurrentState();
  // gripper_group.setNamedTarget("gripper_half_closed");
  // moveit::planning_interface::MoveGroupInterface::Plan plan_gripper;
  // bool success_gripper = (gripper_group.plan(plan_gripper) ==
  //                         moveit::core::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(node->get_logger(),
  //             "Planning to named target gripper_half_closed %s",
  //             success_gripper ? "SUCCEEDED" : "FAILED");
  // if (success_gripper) {
  //   gripper_group.execute(plan_gripper);
  // }

  // arm_group.setStartStateToCurrentState();
  // arm_group.setNamedTarget("pose2");

  // moveit::planning_interface::MoveGroupInterface::Plan plan2;
  // bool success2 =
  //     (arm_group.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(node->get_logger(), "Planning to named target pose2 %s ",
  //             success2 ? "SUCCEEDED" : "FAILED");
  // if (success2) {
  //   arm_group.execute(plan2);
  // }

  // gripper_group.setStartStateToCurrentState();
  // gripper_group.setNamedTarget("gripper_open");
  // moveit::planning_interface::MoveGroupInterface::Plan plan_gripper_open;
  // bool success_gripper_open = (gripper_group.plan(plan_gripper_open) ==
  //                              moveit::core::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(node->get_logger(), "Planning to named target gripper_open %s",
  //             success_gripper_open ? "SUCCEEDED" : "FAILED");
  // if (success_gripper_open) {
  //   gripper_group.execute(plan_gripper_open);
  // }

  // arm_group.setStartStateToCurrentState();
  // arm_group.setNamedTarget("home");
  // moveit::planning_interface::MoveGroupInterface::Plan plan3;
  // bool success3 =
  //     (arm_group.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(node->get_logger(), "Planning to named target home %s ",
  //             success3 ? "SUCCEEDED" : "FAILED");
  // if (success3) {
  //   arm_group.execute(plan3);
  // }

  // -------------------------------------------------------------

  // // Joint goal

  // std::vector<double> joint_group_positions = {1.5, 0.5, 0.0, 1.5, 0.0,
  // -0.7};

  // arm_group.setStartStateToCurrentState();
  // arm_group.setJointValueTarget(joint_group_positions);

  // moveit::planning_interface::MoveGroupInterface::Plan plan_joint_goal;
  // bool success_joint_goal = (arm_group.plan(plan_joint_goal) ==
  //                            moveit::core::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(node->get_logger(), "Planning to joint goal %s",
  //             success_joint_goal ? "SUCCEEDED" : "FAILED");
  // if (success_joint_goal) {
  //   arm_group.execute(plan_joint_goal);
  // }

  // ----------------------------------------------------------------
  // Pose goal

  tf2::Quaternion q;
  q.setRPY(M_PI, 0, 0);  // Roll, Pitch, Yaw
  q = q.normalize();

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.pose.position.x = 0.7;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = 0.4;
  target_pose.pose.orientation.x = q.x();
  target_pose.pose.orientation.y = q.y();
  target_pose.pose.orientation.z = q.z();
  target_pose.pose.orientation.w = q.w();

  arm_group.setStartStateToCurrentState();
  arm_group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan_pose_goal;
  bool success_pose_goal = (arm_group.plan(plan_pose_goal) ==
                            moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(node->get_logger(), "Planning to pose goal %s",
              success_pose_goal ? "SUCCEEDED" : "FAILED");
  if (success_pose_goal) {
    arm_group.execute(plan_pose_goal);
  }

  // Cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose target_pose1 = arm_group.getCurrentPose().pose;
  target_pose1.position.z += -0.2;
  waypoints.push_back(target_pose1);
  geometry_msgs::msg::Pose target_pose2 = target_pose1;
  target_pose2.position.y += 0.2;
  waypoints.push_back(target_pose2);
  geometry_msgs::msg::Pose target_pose3 = target_pose2;
  target_pose3.position.y += -0.2;
  target_pose3.position.z += 0.2;
  waypoints.push_back(target_pose3);

  moveit_msgs::msg::RobotTrajectory trajectory;

  double fraction =
      arm_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  if (fraction >= 1.0 - 1e-6) {
    arm_group.execute(trajectory);
    RCLCPP_INFO(node->get_logger(),
                "Cartesian path computed successfully. Executing...");
  } else {
    RCLCPP_WARN(node->get_logger(),
                "Could not compute a complete Cartesian path. Fraction: %f",
                fraction);
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}