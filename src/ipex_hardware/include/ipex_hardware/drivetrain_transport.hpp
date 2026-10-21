#ifndef IPEX_HARDWARE__DRIVETRAIN_TRANSPORT_HPP_
#define IPEX_HARDWARE__DRIVETRAIN_TRANSPORT_HPP_

#include <array>
#include <chrono>

namespace ipex_hardware
{

// Measured wheel motion from the RoboClaw encoders (SPEED / ENC telemetry).
// Index order (same as CURR): 0 back left, 1 back right, 2 front right, 3 front left.
// Wheel frame: positive = rover forward on that side, same as DRIVE.
struct WheelFeedback
{
  std::array<double, 4> speed_rad_s{};
  std::array<double, 4> position_rad{};
  std::chrono::steady_clock::time_point speed_stamp{};
  std::chrono::steady_clock::time_point position_stamp{};
  bool has_speed = false;
  bool has_position = false;
};

class DrivetrainTransport
{
public:
  virtual ~DrivetrainTransport() = default;

  virtual bool write(
    double left_wheel_rad_s,
    double right_wheel_rad_s) = 0;

  // Immediately stop all drive motors.
  virtual bool stop() = 0;

  // Latest measured wheel motion (has_* false until the firmware sends it).
  virtual WheelFeedback wheel_feedback() const {return WheelFeedback{};}
};

}  // namespace ipex_hardware

#endif
