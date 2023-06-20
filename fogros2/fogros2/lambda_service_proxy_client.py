import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
import pickle
from .util import get_ROS_class


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        service_class = get_ROS_class("example_interfaces/AddTwoInts")
        self.cli = self.create_client(service_class, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self):
        with open("/tmp/pickled_request", "rb") as f:
            self.req = f.read()
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    num1 = int(1)
    num2 = int(2)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (num1, num2, response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
