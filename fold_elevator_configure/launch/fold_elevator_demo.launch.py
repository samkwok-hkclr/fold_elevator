import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
 
 
def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "xacro_file",
            default_value="fold_elevator_demo.urdf.xacro",
            description=""
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "sim",
            default_value="true",
            description="Use simulation"
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "can_interface",
            default_value="can2",
            description="CAN interface"
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "desired_config_update_rate",
            default_value="1000",
            description=""
        )
    )

    moveit_config = (
        MoveItConfigsBuilder("fold_elevator", package_name="fold_elevator_configure")
        .robot_description(file_path="config/fold_elevator_demo.urdf.xacro")
        .robot_description_semantic(file_path="config/fold_elevator.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
 
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    ld.add_action(move_group_node)

    rviz_config = os.path.join(get_package_share_directory("fold_elevator_configure"), "config", "moveit.rviz")
 
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )
    ld.add_action(rviz_node)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    ld.add_action(robot_state_publisher)
 
    ros2_controllers_path = os.path.join(
        get_package_share_directory("fold_elevator_configure"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )
    ld.add_action(ros2_control_node)
 
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    ld.add_action(joint_state_broadcaster_spawner)

    fold_elevator_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "fold_elevator_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    ld.add_action(fold_elevator_controller_spawner)

    return ld