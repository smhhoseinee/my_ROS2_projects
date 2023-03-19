#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from yinyang_msgs.srv import StLen
from yinyang_msgs.action import Farewell
from std_msgs.msg import String
from rclpy.action import ActionServer

class YinNode(Node):

    def __init__(self):
        super().__init__('yinnode')
        #create the action server
        self.action_server = ActionServer(
            self, 
            Farewell,
            'farewell',
            execute_callback=self.execute_callback
        )
        
        # #define the service
        # self.srv = self.create_service(StLen, 'yinnode_service', self.service_callback)       # CHANGE

        #define the client
        # self.service_client_ = self.create_client(StLen, 'yangnode_service')
        
        #define the publisher
        # self.publisher_ = self.create_publisher(String, '/conversation', 10)

        #whether it should send message or wait for Yang
        self.my_turn_to_send = True
        #node has received a bye in its farewell action server
        self.bye_received = False



        
        # while not self.service_client_.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        
        self.req = StLen.Request()
        self.index = 0
        self.msgs = ["I am Yin, some mistake me for an actual material entity but I am more of a concept",
                        "Interesting Yang, so one could say, in a philosophical sense, we are two polar elements",
                        "We, Yang, are therefore the balancing powers in the universe.",
                        "Difficult and easy complete each other.",
                        "Long and short show each other.",
                        "Noise and sound harmonize each other.",
                        "You shine your light."]
        
        self.declare_parameter('shout', True)
        self.declare_parameter('opacity', 100)


    def send_request(self, message):
        self.req.a = message
        self.req.b = len(message)
        future = self.service_client_.call_async(self.req)
        self.get_logger().info('request is sent successfully')
        future.add_done_callback(self.callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        if 'bye' in goal_handle.request.req:
            self.get_logger().info('accepted')
            self.bye_received = True

            self.get_logger().info(goal_handle.request.req)
            feedback_msg = Farewell.Feedback()
            opacity_param = self.get_parameter('opacity')
            opacity_reduction_step = 10
            opacity = opacity_param.value
            while opacity >= 0:
                feedback_msg.opacity = opacity
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.1)
                opacity -= opacity_reduction_step 

            goal_handle.succeed()

            result = Farewell.Result()
            #send a result
            result.res = 'farewell'

            return result


    def callback(self, future):
        try:
            # response = future.result()
            self.index += 1
            # msg_to_be_published = String()
            # msg_to_be_published.data = f"yangnode said: {self.msgs[self.index-1]}, {len(self.msgs[self.index-1])}, {response.sum}"
            # self.publisher_.publish(msg_to_be_published)
            
            if self.index < len(self.msgs) and self.my_turn_to_send:
                shout = self.get_parameter('shout').get_parameter_value().bool_value
                message_to_send = self.msgs[self.index]
                if shout:
                    message_to_send = '**' + message_to_send + '**'

                self.send_request(message_to_send)
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))

    def service_callback(self, request, response):
        response.sum = sum([ord(c) for c in request.a])  # sum of character ASCII values in string 
        self.get_logger().info('Incoming request\na: %s b: %d' % (request.a, request.b))  # log the incoming request
        self.get_logger().info('response\na: %d' % (response.sum))  # log the incoming request

        #publish on /conversation topic
        msg = String()
        msg.data = 'Yang said: '+ request.a + ', ' + str(request.b) + ', ' + str(response.sum)
        self.publisher_.publish(msg)
        return response

        
def main(args=None):
    rclpy.init(args=args)
    yinnode = YinNode()

    # yinnode.send_request(yinnode.msgs[yinnode.index])
    rclpy.spin(yinnode)
    yinnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
