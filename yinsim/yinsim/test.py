#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import StLen
from std_msgs.msg import String

class YinNode(Node):

    def __init__(self):
        super().__init__('yin_node')
        self.publisher_ = self.create_publisher(String, '/conversation', 10)
        self.service_client_ = self.create_client(StLen, 'st_len')
        while not self.service_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = StLen.Request()
        self.index = 0
        self.msgs = ["I am Yin, some mistake me for an actual material entity but I am more of a concept",
                        "Interesting Yang, so one could say, in a philosophical sense, we are
                        two polar elements",
                        "We, Yang, are therefore the balancing powers in the universe.",
                        "Difficult and easy complete each other.",
                        "Long and short show each other.",
                        "Noise and sound harmonize each other.",
                        “You shine your light.”]

    def send_request(self, message):
        self.req.a = message
        self.req.b = len(message)
        future = self.service_client_.call_async(self.req)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.index += 1
            msg = f"yinnode said: {self.msgs[self.index-1]}, {len(self.msgs[self.index-1])}, {response.sum}"
            self.publisher_.publish(msg)
            if self.index < len(self.msgs):
                self.send_request(self.msgs[self.index])
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    yin_node = YinNode()
    yin_node.send_request(yin_node.msgs[yin_node.index])
    rclpy.spin(yin_node)
    yin_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
