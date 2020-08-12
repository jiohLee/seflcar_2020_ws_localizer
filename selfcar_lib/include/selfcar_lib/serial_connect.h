#include <ros/ros.h>
#include <serial/serial.h>

bool valid(serial::Serial& serialPort);

bool connect(std::string port_name, int baud_rate,serial::Serial& serialPort);
