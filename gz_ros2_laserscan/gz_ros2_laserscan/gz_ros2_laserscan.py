import time
import random 
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class Gz_ros2_laserscan(Node):
    def __init__(self):
        super().__init__('gz_ros2_laserscan')
        
        #state 1 means moving forward
        #state 0 means stop
        #state -1 means moving backward
        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.subscription = self.create_subscription(
        #     LaserScan,
        #     '/lidar',
        #     self.listener_callback,
        #     10)
        # self.subscription  # prevent unused variable warning
        self.scann = LaserScan()

        self.call_timer = self.create_timer(2.0, self._timer_cb)

    def listener_callback(self, msg):
        # print(len(msg.ranges)) #len is 2019 from 0-360
        self.state = -1
        self.get_logger().info('Listener received: state is"%d"' % self.state)
 
        self.scann.ranges = msg.ranges[0:72]
        
        for i in range(msg.ranges_size()):
            if msg.ranges(i) < 1.0:
                allMore = False
                break
        if allMore:
            self.state = -1
        else:
            self.state = -1

        current_time = rclpy.Time.now()
        self.scann.header.stamp = current_time
        self.scann.header.frame_id = 'laser'
        self.scann.angle_min = -3.1415
        self.scann.angle_max = 3.1415
        self.scann.angle_increment = 0.00311202858575
        self.scann.time_increment = 4.99999987369e-05
        self.scann.range_min = 0.00999999977648
        self.scann.range_max = 32.0
        self.scann.ranges = msg.ranges[0:72]
        self.scann.intensities = msg.intensities[0:72]
        print(self.scann)
        # pub.publish(self.scann)


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

            # self.state= 1


        #no movement
        elif(self.state == 0):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = 0.0
            move_direction.linear.x = 0.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0

            self.state=0            

        #move backward
        elif(self.state == -1):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = 0.0
            move_direction.linear.x = -20.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0

            self.state=-1            

        #turn right
        elif(self.state == 2):
            move_direction.angular.x = 2.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = 0.0
            move_direction.linear.x = 0.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0

            self.state=1            

        self.get_logger().info('state is"%d"' % self.state)
        self.pub.publish(move_direction)
    
def main(args=None):    
    rclpy.init(args=args)
    gz_ros2_laserscan = Gz_ros2_laserscan()
    gz_ros2_laserscan.state = 1
    rclpy.spin(gz_ros2_laserscan)


    gz_ros2_laserscan.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
