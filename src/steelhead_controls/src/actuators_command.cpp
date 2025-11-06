#define ARDUINO_PORT "/dev/teensy"
#include "steelhead_controls/actuators_command.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
using std::placeholders::_1;

namespace steelhead_controls
{

  ActuatorsCommand::ActuatorsCommand(const rclcpp::NodeOptions & options)
  : Node("actuators_command", options)
  {
    fd_ = open(ARDUINO_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ != -1) {
      struct termios tty;
      tcgetattr (fd_, &tty);
      cfsetospeed (&tty, B115200);
      cfsetispeed (&tty, B115200);
      
      tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
      tty.c_cflag &= ~CSIZE;
      tty.c_cflag |= CS8;         /* 8-bit characters */
      tty.c_cflag &= ~PARENB;     /* no parity bit */
      tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
      tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

      tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
      tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
      tty.c_oflag &= ~OPOST;

      tty.c_cc[VMIN] = 0;
      tty.c_cc[VTIME] = 1;

      service_ = this->create_service<steelhead_interfaces::srv::ActuatorsCommand>(
                  "actuators_command", 
                  std::bind(&ActuatorsCommand::sendOverSerial, this,
                std::placeholders::_1,
                std::placeholders::_2));

      RCLCPP_INFO(this->get_logger(), "Actuators command server succesfully started!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Actuators command server could not connect to arduino! Can the computer recognize the port?");
    }
  }

  ActuatorsCommand::~ActuatorsCommand(){
    close(fd_);
  }

  void ActuatorsCommand::sendOverSerial(const std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Request> request,
          std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Response>      response) {
            if (write(fd_, &request->input, 4) == -1) {
              response->succeeded = false;
            } else {
              response->succeeded = true;
            }
          }

} // namespace steelhead_controls

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto actuators_command_node = std::make_shared<steelhead_controls::ActuatorsCommand>(
        rclcpp::NodeOptions());

    rclcpp::spin(actuators_command_node);

    rclcpp::shutdown();
    return 0;
}