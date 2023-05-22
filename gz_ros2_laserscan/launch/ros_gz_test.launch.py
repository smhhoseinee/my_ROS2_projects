from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('eddiebot_gazebo'),
                             'launch/eddiebot_gz_sim.launch.py')
            ),
            launch_arguments={'world': 'maze_marked'}.items()
        ),

        Node(
            package='gz_ros2_laserscan',
            executable='gz_ros2_laserscan',
            name='gz_ros2_laserscan'
        ), 
        
        Node(
            package='ros_gz_bridge', 
            executable='parameter_bridge', 
            name='parameter_bridge',
            arguments=['/cmd_vel@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge', 
            executable='parameter_bridge', 
            name='parameter_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge', 
            executable='parameter_bridge', 
            name='parameter_bridge',
            arguments=['/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
            output='screen'
        )
        
                
        
    ])