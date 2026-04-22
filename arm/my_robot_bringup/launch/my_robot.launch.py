import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare  # 正确的导入位置


def generate_launch_description():
    # 机械臂描述包
    pkg_name = "my_robot_description"
    xacro_file = "my_robot.urdf.xacro"
    xacro_path = PathJoinSubstitution([FindPackageShare(pkg_name), "urdf", xacro_file])

    robot_description_content = Command(["xacro ", xacro_path])

    # 节点1：robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
    )

    # 动态获取 bringup 包的 share 路径，并构建控制器配置文件路径
    bringup_share = get_package_share_directory("my_robot_bringup")
    controller_config = os.path.join(bringup_share, "config", "ros2_controllers.yaml")

    # 节点2：ros2_control_node，加载控制器配置
    robot_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        parameters=[controller_config],
    )

    # 节点3：关节状态广播器 spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 节点4：手臂控制器 spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # 节点5：夹爪控制器 spawner
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # 节点6：MoveIt move_group 节点
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("my_robot_moveit_config"),
                    "launch",
                    "move_group.launch.py",
                ]
            )
        )
    )

    # 节点7：RViz 可视化
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare(pkg_name), "rviz", "urdf_config.rviz"]
            ),
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            robot_controller_manager,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
            move_group_launch,
            rviz2,
        ]
    )
