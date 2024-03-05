import os #Miscellaneous operating system interfaces
import yaml #import yaml module
from launch import LaunchDescription 
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path): #define funtion load_file
    package_path = get_package_share_directory(package_name) #get package share from ament
    absolute_file_path = os.path.join(package_path, file_path) #use os to generate actual file path

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path): # function for loading yaml i would guess
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file: #pens a file, and returns it as a file object.
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_description_config = load_file(
        "ur5_description", "urdf/ur5.urdf"
    )
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_file(
        "ur5_moveit_config", "config/ur5.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "ur5_moveit_config", "config/kinematics.yaml"
    )

    planning_yaml = load_yaml(
        "ur5_moveit_config", "config/ompl_planning.yaml"
    )

    planning_plugin = {"planning_plugin": "ompl_interface/OMPLPlanner"}

    return LaunchDescription(
        [
            Node(
                package="ur5_planning",
                executable="motion_planning_api_tutorial",
                name="motion_planning_api_tutorial",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    planning_yaml,
                    planning_plugin,
                ],
            )
        ]
    )
