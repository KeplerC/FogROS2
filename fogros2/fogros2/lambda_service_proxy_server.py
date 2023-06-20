
import rclpy
from rclpy.node import Node
import pickle 
from .util import get_ROS_class


class FogROSLambdaService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            get_ROS_class("example_interfaces/AddTwoInts", True), 
            'add_two_ints', 
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        # response.sum = request.a + request.b
        request_serialized = pickle.dumps(request)
        print(request_serialized)
        with open("/tmp/pickled_request", "wb+") as f:
            f.write(request_serialized)

        request = pickle.loads(request_serialized)
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = FogROSLambdaService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()