from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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

    # 节点2：提供 GUI 滑块来发布关节状态（便于手动测试）
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # 节点3：启动 RViz 可视化界面
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

    return LaunchDescription([robot_state_publisher, joint_state_publisher_gui, rviz2])
