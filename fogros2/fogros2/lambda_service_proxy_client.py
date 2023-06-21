import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
import pickle
from .util import get_ROS_class
import codecs

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        service_class = get_ROS_class("example_interfaces/AddTwoInts")
        self.cli = self.create_client(service_class, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self, request):
        
        print(self.req.a, self.req.b)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    request_parameter = sys.argv[1]
    import jsonpickle 
    request = jsonpickle.decode(request_parameter)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(request)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
