#include "ipex_hardware/ipex_arm_system.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"


namespace ipex_hardware
{

namespace
{
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double DEG_TO_RAD = M_PI / 180.0;

bool starts_with(const std::string & s, const char * prefix)
{
  return s.rfind(prefix, 0) == 0;
}

std::string param_or(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, const std::string & fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}
}  // namespace


hardware_interface::CallbackReturn IpexArmSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hw = get_hardware_info().hardware_parameters;

  try {
    arm_name_ = hw.at("arm_name");
    serial_device_ = hw.at("serial_device");
    serial_baud_ = std::stoi(param_or(hw, "serial_baud", "115200"));
    drum_steps_per_rev_ = std::stod(param_or(hw, "drum_steps_per_rev", "400"));
    home_on_activate_ = param_or(hw, "home_on_activate", "false") == "true";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Invalid or missing arm hardware parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (drum_steps_per_rev_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "drum_steps_per_rev must be greater than zero.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Classify joints by command interface.
  for (const auto & joint : get_hardware_info().joints) {
    for (const auto & cmd : joint.command_interfaces) {
      if (cmd.name == "position") {
        if (!shoulder_joint_.empty()) {
          RCLCPP_ERROR(get_logger(), "More than one position joint; expected one shoulder.");
          return hardware_interface::CallbackReturn::ERROR;
        }
        shoulder_joint_ = joint.name;
      } else if (cmd.name == "velocity") {
        drum_joints_.push_back(joint.name);
      }
    }
  }

  if (shoulder_joint_.empty() || drum_joints_.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Arm '%s' needs one position (shoulder) joint and at least one velocity (drum) joint.",
      arm_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(), "Arm '%s': shoulder=%s, %zu drum joint(s), device=%s",
    arm_name_.c_str(), shoulder_joint_.c_str(), drum_joints_.size(), serial_device_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn IpexArmSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  for (const auto & [name, description] : joint_state_interfaces_) {
    (void)description;
    set_state(name, 0.0);
  }
  for (const auto & [name, description] : joint_command_interfaces_) {
    (void)description;
    set_command(name, NaN);
  }

  last_sent_shoulder_rad_ = NaN;
  last_sent_drum_rad_s_ = NaN;
  homed_ = false;
  stop_requested_ = false;
  home_requested_ = false;

  port_ = std::make_unique<SerialPort>(serial_device_, serial_baud_);
  if (!port_->open()) {
    RCLCPP_ERROR(
      get_logger(), "Arm '%s': failed to open %s: %s",
      arm_name_.c_str(), serial_device_.c_str(), port_->last_error().c_str());
    port_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(get_logger(), "Arm hardware node is unavailable.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  homed_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "/ipex/" + arm_name_ + "/homed", rclcpp::QoS(1).transient_local());
  publish_homed(false);

  all_stop_sub_ = node->create_subscription<std_msgs::msg::Empty>(
    "/ipex/all_stop", 10,
    [this](const std_msgs::msg::Empty &) {stop_requested_ = true;});

  home_sub_ = node->create_subscription<std_msgs::msg::Empty>(
    "/ipex/home_arms", 10,
    [this](const std_msgs::msg::Empty &) {home_requested_ = true;});

  RCLCPP_INFO(get_logger(), "Arm '%s' configured on %s.", arm_name_.c_str(),
    serial_device_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn IpexArmSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (home_on_activate_) {
    home_requested_ = true;
    RCLCPP_INFO(get_logger(), "Arm '%s': homing on activate.", arm_name_.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Arm '%s' activated (NOT homed). Home with: "
      "ros2 topic pub --once /ipex/home_arms std_msgs/msg/Empty {}",
      arm_name_.c_str());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn IpexArmSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (port_ && port_->is_open()) {
    port_->write_line("STOP");
  }
  RCLCPP_INFO(get_logger(), "Arm '%s' deactivated (STOP sent).", arm_name_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn IpexArmSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  all_stop_sub_.reset();
  home_sub_.reset();
  homed_pub_.reset();
  port_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}


void IpexArmSystem::publish_homed(bool homed)
{
  if (homed_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = homed;
    homed_pub_->publish(msg);
  }
}


void IpexArmSystem::handle_line(const std::string & line)
{
  if (starts_with(line, "STATE ")) {
    int homed = 0;
    double deg = 0.0;
    char activity[32] = {0};
    long drum_steps_s = 0;

    if (std::sscanf(line.c_str(), "STATE %d %lf %31s %ld",
      &homed, &deg, activity, &drum_steps_s) != 4)
    {
      RCLCPP_WARN(get_logger(), "Arm '%s': malformed STATE: %s", arm_name_.c_str(),
        line.c_str());
      return;
    }

    set_state(shoulder_joint_ + "/position", deg * DEG_TO_RAD);

    const double drum_rad_s =
      static_cast<double>(drum_steps_s) * 2.0 * M_PI / drum_steps_per_rev_;
    for (const auto & joint : drum_joints_) {
      set_state(joint + "/velocity", drum_rad_s);
    }

    if ((homed != 0) != homed_) {
      homed_ = (homed != 0);
      publish_homed(homed_);
    }
    return;
  }

  if (line == "CAL_START") {
    RCLCPP_INFO(get_logger(), "Arm '%s': homing started.", arm_name_.c_str());
  } else if (line == "CAL_DONE") {
    RCLCPP_INFO(get_logger(), "Arm '%s': homing complete.", arm_name_.c_str());
  } else if (line == "CAL_ABORTED") {
    RCLCPP_WARN(get_logger(), "Arm '%s': homing ABORTED (not homed).", arm_name_.c_str());
  } else if (line == "STOPPED") {
    RCLCPP_WARN(get_logger(), "Arm '%s': STOPPED, brake engaged.", arm_name_.c_str());
  } else if (starts_with(line, "ARM_DONE")) {
    RCLCPP_INFO(get_logger(), "Arm '%s': %s", arm_name_.c_str(), line.c_str());
  } else if (starts_with(line, "ERR")) {
    RCLCPP_WARN(get_logger(), "Arm '%s' Teensy: %s", arm_name_.c_str(), line.c_str());
  } else if (starts_with(line, "#")) {
    RCLCPP_DEBUG(get_logger(), "Arm '%s' Teensy: %s", arm_name_.c_str(), line.c_str());
  } else {
    RCLCPP_DEBUG(get_logger(), "Arm '%s' unrecognized: %s", arm_name_.c_str(), line.c_str());
  }
}


hardware_interface::return_type IpexArmSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!port_ || !port_->is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  for (const auto & line : port_->read_lines()) {
    handle_line(line);
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type IpexArmSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!port_ || !port_->is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  const double shoulder_cmd = get_command(shoulder_joint_ + "/position");
  const double drum_cmd = get_command(drum_joints_.front() + "/velocity");

  // ALL STOP: send STOP and treat current commands as already sent, so the
  // arm is not re-commanded until a NEW target arrives.
  if (stop_requested_.exchange(false)) {
    if (!port_->write_line("STOP")) {
      RCLCPP_ERROR(get_logger(), "Arm '%s': failed to send STOP: %s",
        arm_name_.c_str(), port_->last_error().c_str());
      return hardware_interface::return_type::ERROR;
    }
    last_sent_shoulder_rad_ = shoulder_cmd;
    last_sent_drum_rad_s_ = 0.0;
    return hardware_interface::return_type::OK;
  }

  if (home_requested_.exchange(false)) {
    if (!port_->write_line("CAL")) {
      RCLCPP_ERROR(get_logger(), "Arm '%s': failed to send CAL: %s",
        arm_name_.c_str(), port_->last_error().c_str());
      return hardware_interface::return_type::ERROR;
    }
  }

  // Shoulder: send ARM only when the target changes.
  if (std::isfinite(shoulder_cmd) &&
    (std::isnan(last_sent_shoulder_rad_) ||
    std::fabs(shoulder_cmd - last_sent_shoulder_rad_) > 1e-6))
  {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "ARM %.2f", shoulder_cmd * RAD_TO_DEG);
    if (!port_->write_line(buf)) {
      RCLCPP_ERROR(get_logger(), "Arm '%s': failed to send %s: %s",
        arm_name_.c_str(), buf, port_->last_error().c_str());
      return hardware_interface::return_type::ERROR;
    }
    last_sent_shoulder_rad_ = shoulder_cmd;
  }

  // Drums: send DRUM only when the speed changes (continuous on the Teensy).
  if (std::isfinite(drum_cmd) &&
    (std::isnan(last_sent_drum_rad_s_) ||
    std::fabs(drum_cmd - last_sent_drum_rad_s_) > 1e-6))
  {
    const long steps_s =
      std::lround(drum_cmd * drum_steps_per_rev_ / (2.0 * M_PI));
    char buf[48];
    std::snprintf(buf, sizeof(buf), "DRUM %ld", steps_s);
    if (!port_->write_line(buf)) {
      RCLCPP_ERROR(get_logger(), "Arm '%s': failed to send %s: %s",
        arm_name_.c_str(), buf, port_->last_error().c_str());
      return hardware_interface::return_type::ERROR;
    }
    last_sent_drum_rad_s_ = drum_cmd;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ipex_hardware


PLUGINLIB_EXPORT_CLASS(
  ipex_hardware::IpexArmSystem,
  hardware_interface::SystemInterface)
