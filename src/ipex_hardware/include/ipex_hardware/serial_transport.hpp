#ifndef IPEX_HARDWARE__SERIAL_TRANSPORT_HPP_
#define IPEX_HARDWARE__SERIAL_TRANSPORT_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "ipex_hardware/drivetrain_transport.hpp"

namespace ipex_hardware
{

class SerialTransport : public DrivetrainTransport
{
public:
  SerialTransport(
    const rclcpp::Node::SharedPtr & node,
    const std::string & device,
    int baud_rate);

  ~SerialTransport() override;

  bool write(
    double left_wheel_rad_s,
    double right_wheel_rad_s) override;

private:
  bool open_port();
  void close_port();
  void read_available();
  void handle_line(const std::string & line);

  void publish_voltages(
    float back_left,
    float back_right,
    float front_right,
    float front_left);

  void publish_currents(
    float back_left,
    float back_right,
    float front_right,
    float front_left);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr back_left_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr back_right_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_right_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_left_voltage_pub_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr back_left_current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr back_right_current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_right_current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_left_current_pub_;

  std::string device_;
  int baud_rate_;
  int fd_ = -1;

  std::string rx_buffer_;
};

}  // namespace ipex_hardware

#endif
