from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gz_ros2_twist',
            executable='gz_ros2_twist',
            name='gz_ros2_twist'
        ), 
        
        Node(
            package='ros_gz_bridge', 
            executable='parameter_bridge', 
            name='parameter_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            output='screen'
        )        
    ])