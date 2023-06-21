
import rclpy
from rclpy.node import Node
import pickle 
from .util import get_ROS_class
from .aws_lambda import AWSLambdas
import codecs

class FogROSLambdaService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            get_ROS_class("example_interfaces/AddTwoInts", True), 
            'add_two_ints', 
            self.add_two_ints_callback
        )
        self.lambda_function = AWSLambdas()
        # self.lambda_function.create()

    def add_two_ints_callback(self, request, response):
        import jsonpickle
        
        request_serialized = jsonpickle.encode(request) #codecs.encode(pickle.dumps(request), "base64").decode()
        # print(request_serialized)
        # TODO: converts to json or other format
        print(request_serialized)
        final_request = "{\"param\": " + request_serialized + "}"
        print(final_request)
        with open("/tmp/pickled_request", "w+") as f:
            f.write(final_request)


        print(response)
        ret = self.lambda_function.invoke("/tmp/pickled_request")
        print(ret)
        # I don't know why I need to decode twice
        response = jsonpickle.decode(jsonpickle.decode(ret))
        print(response)
        print("response")
        print(response.sum)
        # request = pickle.loads(request_serialized)
        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = FogROSLambdaService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()