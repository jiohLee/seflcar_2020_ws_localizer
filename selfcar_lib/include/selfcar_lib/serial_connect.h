#ifndef SERIAL_CONNECT_H
#define SERIAL_CONNECT_H

#include <ros/ros.h>
#include <serial/serial.h>

static inline bool valid(serial::Serial& serialPort)
{
    if(serialPort.isOpen())
    {
        ROS_INFO("Serial Port Available");
        return true;
    }
    return false;
}

static inline bool connect(std::string port_name, int baud_rate, serial::Serial& serialPort)
{
    if(valid(serialPort))
    {
        ROS_INFO("Cannot Connect USB Device");
        ros::shutdown();
        return false;
    }
    try
    {
        serialPort.setPort(port_name);
        serialPort.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialPort.setTimeout(to);
        serialPort.open();
        ROS_INFO("Serial Port Open");
        return true;
    }
    catch (const std::exception& e)
    {
        ROS_INFO("Cannot Connect USB Debice");
        ros::shutdown();
        return false;
    }
}

#endif
