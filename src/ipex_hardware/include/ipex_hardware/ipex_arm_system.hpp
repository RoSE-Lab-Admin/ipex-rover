#ifndef IPEX_HARDWARE__IPEX_ARM_SYSTEM_HPP_
#define IPEX_HARDWARE__IPEX_ARM_SYSTEM_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

#include "ipex_hardware/serial_port.hpp"

namespace ipex_hardware
{

// ros2_control hardware component for ONE arm Teensy (firmware/arm/arm.ino).
// One instance per arm, each with its own <ros2_control> block.
//
// Joints (from the URDF block):
//   - exactly one joint with a "position" command interface: the shoulder
//   - one or more joints with a "velocity" command interface: the drums
//     (all drums receive the first drum joint's command; they turn in sync)
//
// Commands -> Teensy (only when the command changes):
//   shoulder position [rad] -> "ARM <deg>"
//   drum velocity [rad/s]   -> "DRUM <steps/s>"
// Teensy -> state:
//   "STATE <homed> <deg> <activity> <steps/s>" -> shoulder position, drum velocity
//
// ROS topics (on the hardware component node):
//   /ipex/all_stop            (std_msgs/Empty, sub)  -> "STOP"
//   /ipex/home_arms           (std_msgs/Empty, sub)  -> "CAL"
//   /ipex/<arm_name>/homed    (std_msgs/Bool, pub, transient local)
class IpexArmSystem : public hardware_interface::SystemInterface
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

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  void handle_line(const std::string & line);
  void publish_homed(bool homed);

  // Parameters
  std::string arm_name_;
  std::string serial_device_;
  int serial_baud_ = 115200;
  double drum_steps_per_rev_ = 400.0;
  bool home_on_activate_ = false;

  // Joints
  std::string shoulder_joint_;
  std::vector<std::string> drum_joints_;

  std::unique_ptr<SerialPort> port_;

  // Last values sent to the Teensy (NaN = nothing sent yet)
  double last_sent_shoulder_rad_;
  double last_sent_drum_rad_s_;

  bool homed_ = false;

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> home_requested_{false};

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr all_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr home_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr homed_pub_;
};

}  // namespace ipex_hardware

#endif  // IPEX_HARDWARE__IPEX_ARM_SYSTEM_HPP_
