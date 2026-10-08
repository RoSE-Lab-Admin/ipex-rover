#ifndef IPEX_HARDWARE__SERIAL_PORT_HPP_
#define IPEX_HARDWARE__SERIAL_PORT_HPP_

#include <string>
#include <vector>

namespace ipex_hardware
{

// Non-blocking, newline-delimited serial port (POSIX termios, 8N1, raw).
// Used for Teensy USB serial links.
class SerialPort
{
public:
  SerialPort(const std::string & device, int baud_rate);
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Returns false and fills last_error() on failure.
  bool open();
  void close();
  bool is_open() const {return fd_ >= 0;}

  // Writes line + '\n'. Returns false on failure or partial write.
  bool write_line(const std::string & line);

  // Reads everything available and returns complete lines
  // (without '\r' / '\n'). Incomplete data stays buffered.
  std::vector<std::string> read_lines();

  const std::string & device() const {return device_;}
  const std::string & last_error() const {return last_error_;}

private:
  std::string device_;
  int baud_rate_;
  int fd_ = -1;
  std::string rx_buffer_;
  std::string last_error_;
};

}  // namespace ipex_hardware

#endif  // IPEX_HARDWARE__SERIAL_PORT_HPP_
