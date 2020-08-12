#include <ros/ros.h>
#include <serial/serial.h>

#include <string>
#include <iomanip>
#include <bitset>

#include "selfcar_lib/serial_connect.h"

/* <COMMUNICATION DETIALS RECEIVER>
 *
 * < 'id' 'command' 'data' >
 * with id : send command to imu
 * without id : send command to reciver
 * with id '??' : send command to all imu
 *
 * 1. OUTPUT COMMAND
 * 1.1 Baudrate
 * <sb'data'>
 * data : 8 921600 -> defalut
 * data : 7 921600 / 2
 * data : 6 921600 / 2 /2
 * .
 * .
 * .
 *
 * 1.2 output rate
 * <sor'data'>
 * data : 1~1000[ms]
 *
 * 1.3 output code
 * <soc'data'>
 * data : 1 -> ascii code
 * data : 2 -> hex(bibary) code
 *
 * 1.4 output format
 * @ gyroscope + acceleration + .... = orientation
 * <sof'data'>
 * data : 1 -> euler angle
 *      RPY order, degree, 0.01
 *      R : -180 ~ 180
 *      P : -90 ~ 90
 *      Y : -180 ~ 180
 *      HEX mode -> divide 100
 * data : 2 -> quaternion
 *      z,y,z,w order, 0.0001
 *      HEX mode -> divide 10000
 *
 * 1.5 output gyro
 * <sog'data'>
 * data : 0 -> no output
 * data : 1 -> output
 * x,y,z degree per second
 * HEX mode -> divide 10
 *
 * 1.6 output accelero
 * <soa'data'>
 * x,y,z 1g -> 9.81m/s^2 0.001
 * HEX mode -> divide 1000
 * data 1 : include gravity acceleration
 * data 2 : without gravity acceleration, local base on x,y,z, axis
 * data 3 : without gravity acceleration, global base on n,s,w,e,up,down
 * data 4 : local velocity data m/s
 * data 5 : global velocity data m/s
 *
 * 1.7 output magneto
 * 1.8 outout distance
 * 1.9 output temperature
 * 1.10 output battery
 *
 * 1.11 output time stamp
 *
 * 2. ETC COMMAND
 * 2.1 set RF channel
 * 2.2 set RF max id
 * 2.3 configuration : show configuration details
 *  <cfg>
 * 2.4 power on start
 * 2.5 start
 * 2.6 stop
 * 2.7 load factory settings
 * 2.8 check version
 * <ver>
 */

/*  <COMMUNICATION DETAIL IMU>
 *  < ID 'cg' > gyro calibration
 *  < ID 'cas'> accelerometer calibration
 */

std::string data_full;

void getData(serial::Serial& serialPort)
{
    if(serialPort.available())
    {
        std::string data_byte;
        data_byte = serialPort.read(1);
        std::cout << data_byte ;
    }
}

int main(int argc, char** argv)
{

    ros::init(argc,argv,"ebimu_node_test");
    ros::NodeHandle nh;
    serial::Serial serialPort;
    std::string publish_topic_name;
    std::string port_name;
    int baud_rate;
    ros::Rate rate(20);

    ROS_INFO("init IMU test....");

    nh.param<std::string>("/ebimu_node_test/publish_topic_name",publish_topic_name,"/ebimu_");
    nh.param<int>("/ebimu_node_test/baud_rate",baud_rate,921600);
    nh.param<std::string>("/ebimu_node_test/port_name",port_name,"/dev/ttyUSB0");

    connect(port_name, baud_rate, serialPort);
    serialPort.write("<soc1>"); // output code : ascii
    rate.sleep();
    serialPort.write("<sof1>"); // output format : rpy
    rate.sleep();
    if (nh.param<bool>("/ebimu_node/confirm_settings",true))
    {
        serialPort.write("<sog1>"); // show gyrometer value
        rate.sleep();
        serialPort.write("<soa2>"); // show acclerometer value
        rate.sleep();
        serialPort.write("<sem2>"); // magnetometer on
        rate.sleep();
        serialPort.write("<rha_t0>");   // robust Heading algorithm false
        rate.sleep();
        serialPort.write("<raa_t0>");   // robust Attitude algorithm false
        rate.sleep();
        serialPort.write("<acva_e0>");  // active vibration Cancellation(accel) true
        rate.sleep();
        serialPort.write("<acvg_e0>");  // active vibration Cancellation(gyro) true
        rate.sleep();
        serialPort.write("<rha_l10>");  // RHA level 0.2
        rate.sleep();
        serialPort.write("<posf_sl0.2>"); // pos filter parameter
        rate.sleep();
        if(nh.param<bool>("/ebimu_node/gyrometer_calibration",true))
        {
            ROS_INFO("gyrometer calibration");
            serialPort.write("<cg>");    // gyrometer calibration
            rate.sleep();
        }
        if(nh.param<bool>("/ebimu_node/accelerometer_calibration",true))
        {
            ROS_INFO("accelerometer calibration");
            serialPort.write("<cas>");;    // accelerometer calibration
            rate.sleep();
        }
        if (nh.param<bool>("/ebimu_node/magnetometer_calibration", true))
        {
            ROS_INFO("magnetometer calibration");
            serialPort.write("<cmf>");;    // megnetormeter calibration
            rate.sleep();
        }
        ROS_INFO("set output format");
    }
    while(ros::ok())
    {
        getData(serialPort);
        ros::spinOnce();
    }

}



