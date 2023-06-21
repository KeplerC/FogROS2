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

        self.future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    with open("/tmp/serialized_pickle") as f:
        request_parameter = f.read()
        print(request_parameter)
    import jsonpickle 
    request = jsonpickle.decode(request_parameter)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(request)
    serialized_response = jsonpickle.encode(response)

    with open("/tmp/serialized_response", "w+") as f:
        print(serialized_response)
        f.write(serialized_response)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
