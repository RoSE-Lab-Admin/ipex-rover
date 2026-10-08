#include "ipex_hardware/serial_port.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace ipex_hardware
{

namespace
{
constexpr size_t MAX_LINE_LENGTH = 255;
}  // namespace


SerialPort::SerialPort(const std::string & device, int baud_rate)
: device_(device),
  baud_rate_(baud_rate)
{
  rx_buffer_.reserve(MAX_LINE_LENGTH + 1);
}


SerialPort::~SerialPort()
{
  close();
}


bool SerialPort::open()
{
  close();

  speed_t speed;
  switch (baud_rate_) {
    case 9600: speed = B9600; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    default:
      last_error_ = "unsupported baud rate " + std::to_string(baud_rate_);
      return false;
  }

  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    last_error_ = std::strerror(errno);
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    last_error_ = std::string("tcgetattr: ") + std::strerror(errno);
    close();
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
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    last_error_ = std::string("tcsetattr: ") + std::strerror(errno);
    close();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  rx_buffer_.clear();
  return true;
}


void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}


bool SerialPort::write_line(const std::string & line)
{
  if (fd_ < 0) {
    last_error_ = "port not open";
    return false;
  }

  const std::string out = line + "\n";
  const ssize_t written = ::write(fd_, out.data(), out.size());

  if (written != static_cast<ssize_t>(out.size())) {
    last_error_ = written < 0 ? std::strerror(errno) : "partial write";
    return false;
  }
  return true;
}


std::vector<std::string> SerialPort::read_lines()
{
  std::vector<std::string> lines;
  if (fd_ < 0) {
    return lines;
  }

  char buffer[256];

  while (true) {
    const ssize_t n = ::read(fd_, buffer, sizeof(buffer));

    if (n > 0) {
      for (ssize_t i = 0; i < n; ++i) {
        const char c = buffer[i];
        if (c == '\r') {
          continue;
        }
        if (c == '\n') {
          if (!rx_buffer_.empty()) {
            lines.push_back(rx_buffer_);
            rx_buffer_.clear();
          }
          continue;
        }
        if (rx_buffer_.size() < MAX_LINE_LENGTH) {
          rx_buffer_.push_back(c);
        } else {
          rx_buffer_.clear();  // Oversized/malformed line: drop it
        }
      }
      continue;
    }

    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      last_error_ = std::strerror(errno);
    }
    break;
  }

  return lines;
}

}  // namespace ipex_hardware
