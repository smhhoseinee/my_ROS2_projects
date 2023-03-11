import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.declare_parameter('stop', False)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.state = 1

    def listener_callback(self, msg):
        # Change state based on current x and y
        # if self.state == 2:
        #     self.state = 3
        if  (self.state == 1) and (msg.x > 11.0 or msg.y > 11.0 or msg.x <= 0.0 or msg.y <= 0.0)  :
            self.state = 2
        # elif ():
        #     self.state = 1

    def timer_callback(self):
        msg = Twist()

        # Change state based on stop parameter
        stop = self.get_parameter('stop').get_parameter_value().bool_value
        if stop:
            self.state = 0
        else:
            self.state = 1
        
        # Set twist message based on state
        if self.state == 0:
            # stop
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0

            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

        elif self.state == 1:
            # forward
            msg.linear.x = 2.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0

            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

        elif self.state == 2:
            # backward
            msg.linear.x = -2.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0

            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

            self.state = 3

        elif self.state == 3:
            # turn
            msg.linear.x = 0.0
            msg.linear.y = 2.0
            msg.linear.z = 0.0

            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

            self.state = 1


        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main
