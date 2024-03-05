import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    moveit_config = (
        MoveItConfigsBuilder("ur5")
        .robot_description(
            file_path="config/ur5.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/ur5.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    octomap_updater = {'octomap_frame': 'camera_rgb_optical_frame',
                       'octomap_resolution': 0.05,
                       'max_range': 10.0}
    sensors_config = {'sensors': ['default_sensor','other_sensor'],
                      'default_sensor.sensor_plugin': 'occupancy_map_monitor/PointCloudOctomapUpdater',
                      'default_sensor.point_cloud_topic': '/camera/depth_registered/points',
                      'default_sensor.max_range': 10.0,
                      'default_sensor.point_subsample': 1,
                      'default_sensor.padding_offset': 0.1,
                      'default_sensor.padding_scale': 1.0,
                      'default_sensor.max_update_rate': 1.0,
                      'default_sensor.filtered_cloud_topic': 'filtered_cloud'
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # octomap_updater_config,
            octomap_updater,
            sensors_config,
            ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("ur5_moveit_config"), "launch"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ]
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    static_tf_node_2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_to_camera",
        output="log",
        arguments=["0.0", "0.0", "-1.0", "0.0", "0.0", "0.0", "camera_rgb_optical_frame", "world"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ur5_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    ur5_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur5_arm_controller", "-c", "/controller_manager"],
    )

    ur5_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur5_gripper_controller", "-c", "/controller_manager"],
    )



    return LaunchDescription(
        [
            ros2_control_hardware_type,
            rviz_node,
            static_tf_node,
            static_tf_node_2,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            ur5_arm_controller_spawner,
            ur5_gripper_controller_spawner,
        ]
    )

