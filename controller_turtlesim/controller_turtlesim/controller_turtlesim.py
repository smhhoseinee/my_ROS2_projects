import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtlesim.srv import SetPen
from turtlesim.action import RotateAbsolute
from rclpy.action import ActionClient

import random

import time

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
        # self.action_client = self.create_client(RotateAbsolute, '/turtle1/rotate_absolute')
        self._action_client = ActionClient(self, RotateAbsolute, 'turtle1/rotate_absolute')
        self.delay_while_goal_not_done = False
        self.state = 1
    
    def listener_callback(self, msg):
        # Change state based on current x and y
        if (self.state == 1) and (msg.x > 11.0 or msg.y > 11.0 or msg.x <= 0.0 or msg.y <= 0.0):
            self.get_logger().info('hit state now is 2')
            self.state = 2

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.remaining))

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.delta))
        self.state=1
        self.delay_while_goal_not_done = False


    def timer_callback(self):
        msg = Twist()
        # Change state based on stop parameter
        stop = self.get_parameter('stop').get_parameter_value().bool_value
        if stop:
            self.state = 0
        elif self.state == 0:
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
            msg.linear.x = 1.0
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

            if self.delay_while_goal_not_done == False:
                goal_msg = RotateAbsolute.Goal()
                goal_msg.theta = random.uniform(-5, 5)
                self._action_client.wait_for_server()
                self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
                self._send_goal_future.add_done_callback(self.goal_response_callback)
                self.delay_while_goal_not_done = True

            else :
                dumb_msg = Twist()
                dumb_msg.linear.x = 0.0
                dumb_msg.linear.y = 0.0
                dumb_msg.linear.z = 0.0

                dumb_msg.angular.x = 0.0
                dumb_msg.angular.y = 0.0
                dumb_msg.angular.z = 0.0
                self.publisher_.publish(dumb_msg)

            # # goal = RotateAbsolute.Goal()
            # # self.get_logger().info('Sending goal to rotate turtle')
            # # self._action_client.wait_for_result()
            # # self.get_logger().info('Turtle rotated successfully')

            # self.send_goal(10)
            # self.state = 1


        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear)

    # def send_goal(self, theta):
    #     goal_msg = RotateAbsolute.Goal()
    #     goal_msg.theta = random.uniform(-2.9, 2.9)

    #     self._action_client.wait_for_server()
    #     self.get_logger().info('Sending goal to rotate turtle')
    #         # self._action_client.wait_for_result()
    #         # self.get_logger().info('Turtle rotated successfully')

    #     return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
