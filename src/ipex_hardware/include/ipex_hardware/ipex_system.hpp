#ifndef IPEX_HARDWARE__IPEX_SYSTEM_HPP_
#define IPEX_HARDWARE__IPEX_SYSTEM_HPP_

#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ipex_hardware/drivetrain_transport.hpp"

namespace ipex_hardware
{

class IpexSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::unique_ptr<DrivetrainTransport> drivetrain_transport_;

  double wheel_radius_ = 0.0;
  double wheel_separation_ = 0.0;

  std::string drivetrain_serial_device_;
  int drivetrain_serial_baud_ = 115200;
};

}  // namespace ipex_hardware

#endif
