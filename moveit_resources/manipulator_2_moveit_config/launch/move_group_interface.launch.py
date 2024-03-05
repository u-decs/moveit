from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_manipulator_2").to_moveit_configs()

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="move_group_interface_3",
        package="moveit2_tutorials",
        executable="move_group_interface_3",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )
    move_group_demo_2 = Node(
        name="move_group_interface_4",
        package="moveit2_tutorials",
        executable="move_group_interface_4",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        
    )

    return LaunchDescription(
        [
        move_group_demo,
#        move_group_demo_2,
        ]
        )
