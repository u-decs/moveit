# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_move_group_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("manipulator_2", package_name="moveit_resources_manipulator_2_moveit_config").to_moveit_configs()
#     return generate_move_group_launch(moveit_config)


import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_manipulator_2")
        .robot_description(
            file_path="config/manipulator_2.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },        
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )



    # Start the actual move_group node/action server
    ## BEGIN_SUB_TUTORIAL set_config_move_group
    ## * Add it to the Move Group config
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            # here
        ],
    )
    ## END_SUB_TUTORIAL

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_resources_manipulator_2_moveit_config") + "/config/moveit_move_group.rviz"
    )

    ## BEGIN_SUB_TUTORIAL set_config_rviz
    ## * and to the RViz config
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            # {"use_sim_time": True},
            # here

        ],
    )
    ## END_SUB_TUTORIAL

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description,
                    {"use_sim_time": True},
                    ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_manipulator_2_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[
            {"use_sim_time": True},
            ],
    )

    manipulator_2_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_2_arm_controller", "-c", "/controller_manager"],
    )
    # Load controllers
    # load_controllers = []
    # for controller in [
    #     "manipulator_2_arm_controller",
    #     "manipulator_2_gripper_controller",
    #     "joint_state_broadcaster",
    # ]:
    #     load_controllers += [
    #         ExecuteProcess(
    #             cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
    #             shell=True,
    #             output="screen",
    #         )
    #     ]

    # Warehouse mongodb server
    ## BEGIN_SUB_TUTORIAL start_mongodb_server
    ## * Optionally, start the MongoDB server (uncomment if necessary)
    # mongodb_server_node = Node(
    #    package="warehouse_ros_mongo",
    #    executable="mongo_wrapper_ros.py",
    #    parameters=[
    #        warehouse_ros_config,
    #    ],
    #    output="screen",
    # )
    ## END_SUB_TUTORIAL

    return LaunchDescription(
        [
            ros2_control_hardware_type,
            rviz2_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            manipulator_2_arm_controller_spawner,
        ]
        # + load_controllers
    )


## BEGIN_TUTORIAL
## CALL_SUB_TUTORIAL add_config
## CALL_SUB_TUTORIAL set_config_move_group
## CALL_SUB_TUTORIAL set_config_rviz
## CALL_SUB_TUTORIAL start_mongodb_server
## END_TUTORIAL
