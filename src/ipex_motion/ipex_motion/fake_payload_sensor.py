import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float64


class FakePayloadSensor(Node):

    def __init__(self):
        super().__init__('fake_payload_sensor')

        self.mass = 0.0

        self.excavating = False
        self.depositing = False

        self.mass_rate = 0.25       # kg/s
        self.dt = 0.1               # 10 Hz

        self.mass_pub = self.create_publisher(
            Float64,
            '/payload_mass',
            10
        )

        self.excavation_sub = self.create_subscription(
            Bool,
            '/excavation_active',
            self.excavation_callback,
            10
        )

        self.deposit_sub = self.create_subscription(
            Bool,
            '/deposit_active',
            self.deposit_callback,
            10
        )

        self.reset_sub = self.create_subscription(
            Bool,
            '/payload_reset',
            self.reset_callback,
            10
        )

        self.timer = self.create_timer(
            self.dt,
            self.update_mass
        )

        self.get_logger().info(
            'Fake payload sensor ready: 0.25 kg/s'
        )


    def excavation_callback(self, msg):
        self.excavating = msg.data

    def deposit_callback(self,msg):
        self.depositing = msg.data

    def reset_callback(self, msg):
        if msg.data:
            self.mass = 0.0


    def update_mass(self):

        if self.excavating and not self.depositing:
            self.mass += self.mass_rate * self.dt

        elif self.depositing and not self.excavating:
            self.mass -= self.mass_rate * self.dt


            self.mass = max(0.0, self.mass)

        msg = Float64()
        msg.data = self.mass

        self.mass_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = FakePayloadSensor()

    try:
        rclpy.spin(node)

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
