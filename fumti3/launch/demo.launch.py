from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node 
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from moveit_configs_utils.launches import DeclareBooleanLaunchArg

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("model", package_name="fumti3").to_moveit_configs()
    # moveit_config.to_dict().update({'use_sim_time':True})
    ld =  generate_demo_launch(moveit_config)

    ld.add_action(
        DeclareBooleanLaunchArg(
            "use_sim_time",
            default_value=True,
            description="By default, we are not in debug mode",
        )
    )

    # pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    return ld
