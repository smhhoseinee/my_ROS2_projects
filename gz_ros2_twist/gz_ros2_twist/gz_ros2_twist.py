import time
import random 
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist


class Gz_ros2_twist(Node):
    def __init__(self):
        super().__init__('gz_ros2_twist')
        
        #state 1 means moving forward
        #state 0 means stop
        #state -1 means moving backward
        self.state = 1
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.call_timer = self.create_timer(2.0, self._timer_cb)

    def _timer_cb(self):
        
        move_direction = Twist()

        #move forward
        if (self.state == 1):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = 0.0
            move_direction.linear.x = 2.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0

            self.state= 0


        #no movement
        elif(self.state == 0):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = 0.0
            move_direction.linear.x = 0.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0

            self.state=-1            


        #move backward
        elif(self.state == -1):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = 0.0
            move_direction.linear.x = -2.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0

            self.state=1            


        self.pub.publish(move_direction)
    
def main(args=None):    
    rclpy.init(args=args)
    gz_ros2_twist = Gz_ros2_twist()
    rclpy.spin(gz_ros2_twist)
    gz_ros2_twist.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
