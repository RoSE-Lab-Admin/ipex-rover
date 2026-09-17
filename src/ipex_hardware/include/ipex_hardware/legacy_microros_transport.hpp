#ifndef IPEX_HARDWARE__LEGACY_MICROROS_TRANSPORT_HPP_
#define IPEX_HARDWARE__LEGACY_MICROROS_TRANSPORT_HPP_

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ipex_hardware/drivetrain_transport.hpp"

namespace ipex_hardware
{

class LegacyMicroRosTransport : public DrivetrainTransport
{
public:
  LegacyMicroRosTransport(
    const rclcpp::Node::SharedPtr & node,
    double wheel_radius,
    double wheel_separation);

  bool write(
    double left_wheel_rad_s,
    double right_wheel_rad_s) override;

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  double wheel_radius_;
  double wheel_separation_;
};

}  // namespace ipex_hardware

#endif
