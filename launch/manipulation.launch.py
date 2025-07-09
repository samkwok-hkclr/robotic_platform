import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    params_file = LaunchConfiguration("params_file")
    collision_objects_file = LaunchConfiguration("collision_objects_file")

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(get_package_share_directory("robotic_platform"), "params", "collision_planner_config.yaml"),
        description="",
    )

    # declare_collision_objects_file_cmd = DeclareLaunchArgument(
    #     "collision_objects_file",
    #     default_value=os.path.join(get_package_share_directory("robotic_platform"), "params", "collision_objects.yaml"),
    #     description="",
    # )

    print(os.path.join(get_package_share_directory("robotic_platform"), "params", "collision_objects.yaml"))

    ld.add_action(declare_params_file_cmd)
    # ld.add_action(declare_collision_objects_file_cmd)

    manipulation_server = Node(
        package='robotic_platform',
        # namespace="",
        # name="manipulation_server",
        executable='manipulation_server',
        parameters=[
            params_file,
            {
                "collision_objects_file": os.path.join(
                    get_package_share_directory("robotic_platform"), 
                    "params", 
                    "collision_objects.yaml"),
            }
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', "info"],
        emulate_tty=True,
    )
    ld.add_action(manipulation_server)

    return ld