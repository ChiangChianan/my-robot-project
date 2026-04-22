from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 1. 指定你的机器人描述功能包名和 xacro 文件路径
    pkg_name = "my_robot_description"
    xacro_file = "my_robot.urdf.xacro"
    xacro_path = PathJoinSubstitution([FindPackageShare(pkg_name), "urdf", xacro_file])

    # 2. 通过命令行调用 xacro 工具，生成 URDF 字符串
    robot_description_content = Command(["xacro ", xacro_path])

    # 节点1：发布机器人运动学和 TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
    )

    # 节点2：启动 controller_manager节点，并加载和启动控制器
    robot_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        parameters=[
            "/home/nan/project/ros2practice/my_robot_ws/src/my_robot_bringup/config/ros2_controllers.yaml"
        ],
    )

    # 节点3：加载并启动关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 节点4：加载并启动手臂控制器
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # 节点5：加载并启动末端执行器控制器
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # 节点6：启动 move_group 节点（如果你使用 MoveIt! 进行运动规划）
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

    # 节点7：启动 RViz 可视化界面
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
