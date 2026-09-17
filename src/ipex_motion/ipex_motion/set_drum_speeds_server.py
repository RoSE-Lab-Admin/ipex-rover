import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

from ipex_interfaces.srv import SetDrumSpeeds


class SetDrumSpeedsServer(Node):

    def __init__(self):
        super().__init__('set_drum_speeds_server')

        self.front_pub = self.create_publisher(
            Float64,
            '/front_drum/speed',
            10
        )

        self.rear_pub = self.create_publisher(
            Float64,
            '/rear_drum/speed',
            10
        )

        self.service = self.create_service(
            SetDrumSpeeds,
            '/set_drum_speeds',
            self.set_speeds_callback
        )

        self.get_logger().info(
            'SetDrumSpeeds service ready'
        )


    def set_speeds_callback(self, request, response):

        front = Float64()
        front.data = request.front_rad_s

        rear = Float64()
        rear.data = request.rear_rad_s

        self.front_pub.publish(front)
        self.rear_pub.publish(rear)

        response.success = True
        response.message = (
            f'Drum speeds set: '
            f'front={request.front_rad_s:.2f} rad/s, '
            f'rear={request.rear_rad_s:.2f} rad/s'
        )

        return response


def main(args=None):

    rclpy.init(args=args)

    node = SetDrumSpeedsServer()

    try:
        rclpy.spin(node)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

