from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gz_ros2_laserscan',
            executable='gz_ros2_laserscan',
            name='gz_ros2_laserscan'
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