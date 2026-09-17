#include "ipex_hardware/legacy_microros_transport.hpp"

namespace ipex_hardware
{

LegacyMicroRosTransport::LegacyMicroRosTransport(
  const rclcpp::Node::SharedPtr & node,
  double wheel_radius,
  double wheel_separation)
: wheel_radius_(wheel_radius),
  wheel_separation_(wheel_separation)
{
  cmd_vel_pub_ =
    node->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);
}


bool LegacyMicroRosTransport::write(
  double left_wheel_rad_s,
  double right_wheel_rad_s)
{
  const double linear =
    wheel_radius_ *
    (left_wheel_rad_s + right_wheel_rad_s) /
    2.0;

  const double angular =
    wheel_radius_ *
    (right_wheel_rad_s - left_wheel_rad_s) /
    wheel_separation_;

  geometry_msgs::msg::Twist msg;

  msg.linear.x = linear;

  // TEMPORARY COMPATIBILITY WITH CURRENT DRIVETRAIN FIRMWARE.
  //
  // The existing Teensy negates angular.z before performing
  // differential-drive mixing. Undo that convention here so the
  // rest of the ROS stack retains standard ROS turn semantics.
  msg.angular.z = -angular;

  cmd_vel_pub_->publish(msg);

  return true;
}

}  // namespace ipex_hardware
