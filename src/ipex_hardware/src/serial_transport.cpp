#include "ipex_hardware/serial_transport.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace ipex_hardware
{

SerialTransport::SerialTransport(
  const rclcpp::Node::SharedPtr & node,
  const std::string & device,
  int baud_rate)
: node_(node),
  device_(device),
  baud_rate_(baud_rate)
{
  rx_buffer_.reserve(256);

  back_left_voltage_pub_ =
    node_->create_publisher<std_msgs::msg::Float32>("/rover/back_left/voltage", 10);
  back_right_voltage_pub_ =
    node_->create_publisher<std_msgs::msg::Float32>("/rover/back_right/voltage", 10);
  front_right_voltage_pub_ =
    node_->create_publisher<std_msgs::msg::Float32>("/rover/front_right/voltage", 10);
  front_left_voltage_pub_ =
    node_->create_publisher<std_msgs::msg::Float32>("/rover/front_left/voltage", 10);

  back_left_current_pub_ =
    node_->create_publisher<std_msgs::msg::Float32>("/rover/back_left/current", 10);
  back_right_current_pub_ =
    node_->create_publisher<std_msgs::msg::Float32>("/rover/back_right/current", 10);
  front_right_current_pub_ =
    node_->create_publisher<std_msgs::msg::Float32>("/rover/front_right/current", 10);
  front_left_current_pub_ =
    node_->create_publisher<std_msgs::msg::Float32>("/rover/front_left/current", 10);

  open_port();
}


SerialTransport::~SerialTransport()
{
  if (fd_ >= 0)
  {
    const char stop_command[] = "DRIVE 0.000 0.000\n";
    ::write(fd_, stop_command, sizeof(stop_command) - 1);
  }

  close_port();
}


bool SerialTransport::open_port()
{
  fd_ = ::open(
    device_.c_str(),
    O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd_ < 0)
  {
    std::fprintf(
      stderr,
      "Failed to open serial device %s: %s\n",
      device_.c_str(),
      std::strerror(errno));

    return false;
  }

  termios tty{};

  if (tcgetattr(fd_, &tty) != 0)
  {
    std::fprintf(
      stderr,
      "Failed to read serial settings for %s: %s\n",
      device_.c_str(),
      std::strerror(errno));

    close_port();
    return false;
  }

  speed_t speed;

  switch (baud_rate_)
  {
    case 115200:
      speed = B115200;
      break;

    default:
      std::fprintf(
        stderr,
        "Unsupported serial baud rate: %d\n",
        baud_rate_);

      close_port();
      return false;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  tty.c_iflag &=
    ~(IXON | IXOFF | IXANY | ICRNL | INLCR);

  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    std::fprintf(
      stderr,
      "Failed to configure serial device %s: %s\n",
      device_.c_str(),
      std::strerror(errno));

    close_port();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}


void SerialTransport::close_port()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}


void SerialTransport::publish_voltages(
  float back_left,
  float back_right,
  float front_right,
  float front_left)
{
  std_msgs::msg::Float32 msg;

  msg.data = back_left;
  back_left_voltage_pub_->publish(msg);

  msg.data = back_right;
  back_right_voltage_pub_->publish(msg);

  msg.data = front_right;
  front_right_voltage_pub_->publish(msg);

  msg.data = front_left;
  front_left_voltage_pub_->publish(msg);
}

void SerialTransport::publish_currents(
  float back_left,
  float back_right,
  float front_right,
  float front_left)
{
  std_msgs::msg::Float32 msg;

  msg.data = back_left;
  back_left_current_pub_->publish(msg);

  msg.data = back_right;
  back_right_current_pub_->publish(msg);

  msg.data = front_right;
  front_right_current_pub_->publish(msg);

  msg.data = front_left;
  front_left_current_pub_->publish(msg);
}


void SerialTransport::handle_line(const std::string & line)
{
  float back_left = 0.0f;
  float back_right = 0.0f;
  float front_right = 0.0f;
  float front_left = 0.0f;

  if (std::sscanf(
      line.c_str(),
      "VOLT %f %f %f %f",
      &back_left,
      &back_right,
      &front_right,
      &front_left) == 4)
  {
    publish_voltages(
      back_left,
      back_right,
      front_right,
      front_left);
   return;
  }

  if (std::sscanf(
      line.c_str(),
      "CURR %f %f %f %f",
      &back_left,
      &back_right,
      &front_right,
      &front_left) == 4)
  {
    publish_currents(
      back_left,
      back_right,
      front_right,
      front_left);
    return;
  }

  // CAL_CURRENT START / CAL_CURRENT OK are intentionally ignored for now.
  // A future ROS calibration primitive can promote those status lines into
  // an explicit request/acknowledgement API.

}


void SerialTransport::read_available()
{
  if (fd_ < 0)
  {
    return;
  }

  char buffer[256];

  while (true)
  {
    const ssize_t bytes_read = ::read(fd_, buffer, sizeof(buffer));

    if (bytes_read > 0)
    {
      for (ssize_t i = 0; i < bytes_read; ++i)
      {
        const char c = buffer[i];

        if (c == '\r')
        {
          continue;
        }

        if (c == '\n')
        {
          if (!rx_buffer_.empty())
          {
            handle_line(rx_buffer_);
            rx_buffer_.clear();
          }
          continue;
        }

        if (rx_buffer_.size() < 255)
        {
          rx_buffer_.push_back(c);
        }
        else
        {
          // Discard an oversized/malformed telemetry line.
          rx_buffer_.clear();
        }
      }

      continue;
    }

    if (bytes_read == 0)
    {
      break;
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK)
    {
      break;
    }

    std::fprintf(
      stderr,
      "Failed to read serial device %s: %s\n",
      device_.c_str(),
      std::strerror(errno));
    break;
  }
}


bool SerialTransport::write(
  double left_wheel_rad_s,
  double right_wheel_rad_s)
{
  if (fd_ < 0)
  {
    return false;
  }

  // Drain telemetry every ros2_control cycle so the Teensy's TX buffer does
  // not fill, and republish any complete VOLT lines into ROS.
  read_available();

  char buffer[128];

  const int length = std::snprintf(
    buffer,
    sizeof(buffer),
    "DRIVE %.4f %.4f\n",
    left_wheel_rad_s,
    right_wheel_rad_s);

  if (length <= 0)
  {
    return false;
  }

  const ssize_t bytes_written =
    ::write(fd_, buffer, static_cast<size_t>(length));

  read_available();

  return bytes_written == length;
}

}  // namespace ipex_hardware
