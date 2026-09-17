#include "ipex_hardware/ipex_system.hpp"

#include <string>

#include "ipex_hardware/serial_transport.hpp"
#include "pluginlib/class_list_macros.hpp"


namespace ipex_hardware
{

hardware_interface::CallbackReturn IpexSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hardware_params =
    get_hardware_info().hardware_parameters;

  try
  {
    wheel_radius_ =
      std::stod(hardware_params.at("wheel_radius"));

    wheel_separation_ =
      std::stod(hardware_params.at("wheel_separation"));

    drivetrain_serial_device_ =
      hardware_params.at("drivetrain_serial_device");

    drivetrain_serial_baud_ =
      std::stoi(hardware_params.at("drivetrain_serial_baud"));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid or missing IPEX hardware parameter: %s",
      e.what());

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (wheel_radius_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "wheel_radius must be greater than zero.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (wheel_separation_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "wheel_separation must be greater than zero.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (drivetrain_serial_baud_ <= 0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "drivetrain_serial_baud must be greater than zero.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(),
    "IPEX hardware interface initialized.");

  RCLCPP_INFO(
    get_logger(),
    "Wheel radius: %.3f m | Wheel separation: %.3f m",
    wheel_radius_,
    wheel_separation_);

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn IpexSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  // Initialize all state interfaces to zero.
  for (const auto & [name, description] : joint_state_interfaces_)
  {
    (void)description;
    set_state(name, 0.0);
  }

  // Initialize all command interfaces to zero.
  for (const auto & [name, description] : joint_command_interfaces_)
  {
    (void)description;
    set_command(name, 0.0);
  }

  if (!get_node())
  {
    RCLCPP_ERROR(
      get_logger(),
      "IPEX hardware node is unavailable.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  drivetrain_transport_ =
    std::make_unique<SerialTransport>(
      get_node(),
      drivetrain_serial_device_,
      drivetrain_serial_baud_);

  RCLCPP_INFO(
    get_logger(),
    "Using direct serial drivetrain transport: %s at %d baud.",
    drivetrain_serial_device_.c_str(),
    drivetrain_serial_baud_);

  RCLCPP_INFO(
    get_logger(),
    "IPEX hardware interface configured.");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn IpexSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    get_logger(),
    "IPEX hardware interface activated.");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn IpexSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    get_logger(),
    "IPEX hardware interface deactivated.");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type IpexSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  //
  // V0 MOCK STATE BEHAVIOR
  //
  // Physical sensor feedback is not implemented yet.
  //
  // Future versions will:
  //   - read drivetrain feedback
  //   - read shoulder encoder state
  //   - read drum state
  //   - read voltage/current telemetry
  //   - update ros2_control state interfaces
  //


  // --------------------------------------------------
  // SHOULDERS
  // --------------------------------------------------
  //
  // TEMPORARY:
  // Pretend commanded position is measured position.
  //
  // Remove this when real shoulder feedback is available.

  set_state(
    "front_shoulder_rev/position",
    get_command("front_shoulder_rev/position"));

  set_state(
    "back_shoulder_rev/position",
    get_command("back_shoulder_rev/position"));


  // --------------------------------------------------
  // DRUMS
  // --------------------------------------------------
  //
  // TEMPORARY:
  // Pretend commanded velocity is measured velocity.
  //
  // Remove this when real drum feedback is available.

  set_state(
    "front_left_drum_rev/velocity",
    get_command("front_left_drum_rev/velocity"));

  set_state(
    "front_right_drum_rev/velocity",
    get_command("front_right_drum_rev/velocity"));

  set_state(
    "back_left_drum_rev/velocity",
    get_command("back_left_drum_rev/velocity"));

  set_state(
    "back_right_drum_rev/velocity",
    get_command("back_right_drum_rev/velocity"));


  // --------------------------------------------------
  // DRIVETRAIN
  // --------------------------------------------------
  //
  // TEMPORARY:
  // No wheel encoder feedback exists yet.
  //
  // Pretend commanded wheel velocity is measured velocity,
  // then integrate velocity to create mock wheel position.
  //
  // This is useful for ros2_control testing but is NOT
  // real rover odometry.

  const double dt = period.seconds();

  const char * wheel_names[] = {
    "front_left_wheel_rev",
    "back_left_wheel_rev",
    "front_right_wheel_rev",
    "back_right_wheel_rev"
  };

  for (const char * joint : wheel_names)
  {
    const std::string velocity_name =
      std::string(joint) + "/velocity";

    const std::string position_name =
      std::string(joint) + "/position";

    const double velocity =
      get_command(velocity_name);

    set_state(
      velocity_name,
      velocity);

    const double old_position =
      get_state(position_name);

    set_state(
      position_name,
      old_position + velocity * dt);
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type IpexSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  //
  // DRIVETRAIN COMMAND PATH
  //
  // diff_drive_controller
  //        ↓
  // four ros2_control wheel velocity interfaces
  //        ↓
  // IpexSystem
  //        ↓
  // DrivetrainTransport
  //
  // Wheel velocity commands are sent directly to the drivetrain
  // Teensy over USB serial through SerialTransport.
  //


  // Get wheel velocity commands from ros2_control.

  const double front_left =
    get_command("front_left_wheel_rev/velocity");

  const double back_left =
    get_command("back_left_wheel_rev/velocity");

  const double front_right =
    get_command("front_right_wheel_rev/velocity");

  const double back_right =
    get_command("back_right_wheel_rev/velocity");


  // The rover has two wheels on each side.
  // diff_drive_controller commands both wheels on each side.
  //
  // Average each pair into one semantic left/right command
  // for the drivetrain transport.

  const double left_velocity =
    (front_left + back_left) / 2.0;

  const double right_velocity =
    (front_right + back_right) / 2.0;


  if (!drivetrain_transport_)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Drivetrain transport is not configured.");

    return hardware_interface::return_type::ERROR;
  }


  if (!drivetrain_transport_->write(
      left_velocity,
      right_velocity))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to write drivetrain command.");

    return hardware_interface::return_type::ERROR;
  }


  // Shoulder and drum physical writes are intentionally
  // not implemented in V0.
  //
  // Their ros2_control interfaces exist so the software
  // architecture is ready when the hardware is finalized.


  return hardware_interface::return_type::OK;
}

}  // namespace ipex_hardware


PLUGINLIB_EXPORT_CLASS(
  ipex_hardware::IpexSystem,
  hardware_interface::SystemInterface)
