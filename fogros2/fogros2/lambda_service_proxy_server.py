

from pydoc import locate

import rclpy
from rclpy.node import Node

def get_ROS_class(ros_message_type, srv=True):
    """
    Returns the ROS message class from ros_message_type.
    :return AnyMsgClass: Class of the ROS message.
    """
    try:
        package_name, message_name = ros_message_type.split('/')
    except ValueError:
        raise ValueError(
            'ros_message_type should be in the shape of package_msgs/Message' +
            ' (it was ' + ros_message_type + ')')
    if not srv:
        msg_class = locate('{}.msg.{}'.format(package_name, message_name))
    else:
        msg_class = locate('{}.srv.{}'.format(package_name, message_name))
    if msg_class is None:
        if srv:
            msg_or_srv = '.srv'
        else:
            msg_or_srv = '.msg'
        raise ValueError(
            'ros_message_type could not be imported. (' +
            ros_message_type + ', as "from ' + package_name +
            msg_or_srv + ' import ' + message_name + '" failed.')
    return msg_class



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
        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        print(request)
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = FogROSLambdaService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()