#ifndef IPEX_HARDWARE__DRIVETRAIN_TRANSPORT_HPP_
#define IPEX_HARDWARE__DRIVETRAIN_TRANSPORT_HPP_

namespace ipex_hardware
{

class DrivetrainTransport
{
public:
  virtual ~DrivetrainTransport() = default;

  virtual bool write(
    double left_wheel_rad_s,
    double right_wheel_rad_s) = 0;
};

}  // namespace ipex_hardware

#endif
