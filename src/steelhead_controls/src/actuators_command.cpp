#define ARDUINO_PORT "/dev/ttyACM0"
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
      // sets up connections to arduino
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

      // sets up map defined in actuators_config.yaml
      this->declare_parameter("actuators_names", std::vector<std::string>{});
      this->declare_parameter("actuators_pins", std::vector<long int>{});
      std::vector<std::string> nameList;
      std::vector<long int> pinList;
      this->get_parameter("actuators_names", nameList);
      this->get_parameter("actuators_pins", pinList);

      if (nameList.size() != pinList.size())
        RCLCPP_ERROR(this->get_logger(), "Actuators config file has mismatching name and pin lengths!");

      for (size_t i = 0; i < nameList.size(); i++)
        nameToPin[nameList[i]] = pinList[i];

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
            if (nameToPin.find(request->input) == nameToPin.end()) {
              RCLCPP_ERROR(this->get_logger(), request->input + " is not configured in steelhead_controls actuators_config.yaml!");
              return;
            }

            std::string returnMessage = "Writing " + request->input + " on pin " + std::to_string(nameToPin[request->input]) + " was ";
            if (write(fd_, &nameToPin[request->input], 4) == -1) {
              response->succeeded = false;
              returnMessage+="unsuccessful";
            } else {
              response->succeeded = true;
              returnMessage+="successful";
            }
            RCLCPP_INFO(this->get_logger(), returnMessage);
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