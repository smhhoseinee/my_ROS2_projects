# from tutorial_interfaces.srv import StLen                                                           # CHANGE
from yinyang_msgs.srv import StLen                                                           # CHANGE

import rclpy
from rclpy.node import Node


class MinimalService(Node):

   def __init__(self):
       super().__init__('minimal_service')
       self.srv = self.create_service(StLen, 'st_len', self.st_len_callback)       # CHANGE

   def st_len_callback(self, request, response):
#        response.sum = request.a + request.b + request.c                                                   # CHANGE
       response.sum = sum([ord(c) for c in request.a])  # sum of character ASCII values in string 'a'
#        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c))  # CHANGE
       self.get_logger().info('Incoming request\na: %s b: %d' % (request.a, request.b))  # log the incoming request
       self.get_logger().info('response\na: %d' % (response.sum))  # log the incoming request

       return response

def main(args=None):
   rclpy.init(args=args)

   minimal_service = MinimalService()

   rclpy.spin(minimal_service)

   rclpy.shutdown()

if __name__ == '__main__':
   main()



# from example_interfaces.srv import AddTwoInts

# import rclpy
# from rclpy.node import Node


# class MinimalService(Node):

#     def __init__(self):
#         super().__init__('minimal_service')
#         self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

#     def add_two_ints_callback(self, request, response):
#         response.sum = request.a + request.b
#         self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

#         return response


# def main():
#     rclpy.init()

#     minimal_service = MinimalService()

#     rclpy.spin(minimal_service)

#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
