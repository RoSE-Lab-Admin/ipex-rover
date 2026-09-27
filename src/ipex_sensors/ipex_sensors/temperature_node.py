from smbus2 import SMBus

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature


I2C_BUS = 1

BATTERY_ADDR = 0x19
AVIONICS_ADDR = 0x18

MCP9808_TEMP_REG = 0x05


class TemperatureNode(Node):

    def __init__(self):
        super().__init__('temperature_node')

        self.bus = SMBus(I2C_BUS)

        self.avionics_pub = self.create_publisher(
            Temperature,
            '/rover/avionics/temperature',
            10
        )

        self.battery_pub = self.create_publisher(
            Temperature,
            '/rover/battery/temperature',
            10
        )

        # Publish at 1 Hz
        self.timer = self.create_timer(
            1.0,
            self.publish_temperatures
        )

        self.get_logger().info(
            'MCP9808 temperature node started '
            '(avionics=0x19, battery=0x18)'
        )

    def read_temperature(self, address):
        data = self.bus.read_i2c_block_data(
            address,
            MCP9808_TEMP_REG,
            2
        )

        raw = (data[0] << 8) | data[1]

        temp_c = (raw & 0x0FFF) / 16.0

        if raw & 0x1000:
            temp_c -= 256.0

        return temp_c

    def publish_temperatures(self):
        try:
            avionics_temp = self.read_temperature(AVIONICS_ADDR)
            battery_temp = self.read_temperature(BATTERY_ADDR)

            now = self.get_clock().now().to_msg()

            avionics_msg = Temperature()
            avionics_msg.header.stamp = now
            avionics_msg.header.frame_id = 'avionics_box'
            avionics_msg.temperature = avionics_temp
            avionics_msg.variance = 0.0

            battery_msg = Temperature()
            battery_msg.header.stamp = now
            battery_msg.header.frame_id = 'battery_box'
            battery_msg.temperature = battery_temp
            battery_msg.variance = 0.0

            self.avionics_pub.publish(avionics_msg)
            self.battery_pub.publish(battery_msg)

        except OSError as error:
            self.get_logger().error(
                f'I2C temperature read failed: {error}'
            )

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = TemperatureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
          rclpy.shutdown()


if __name__ == '__main__':
    main()
