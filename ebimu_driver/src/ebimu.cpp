#include "ebimu_driver/ebimu.h"
#include "selfcar_lib/serial_connect.h"
#include "selfcar_lib/covariance.h"

#include <string>

const float_t ebimu::DegreetoRadian = 0.0174533;
const float_t ebimu::gtometerpersec = 9.81;

ebimu::ebimu()
    : pnh_("~")
{
    ROS_INFO("init IMU....");
    std::string publish_topic_name;
    std::string port_name;
    int baud_rate;
    pnh_.param<std::string>("publish_topic_name",publish_topic_name,"/ebimu_");
    pnh_.param<int>("baud_rate",baud_rate,921600);
    pnh_.param<std::string>("port_name",port_name,"/dev/ttyUSB0");
    pnh_.param<int>("covariance_sample_num",sampleNum,3);

    ROS_INFO("port_name : %s", port_name.c_str());
    ROS_INFO("baud_rate : %d", baud_rate);

    connect(port_name, baud_rate, serialPort);

    ros::Rate rate(20);

    serialPort.write("<soc2>"); // output code : hex
    rate.sleep();
    serialPort.write("<sof2>"); // output format : quat
    rate.sleep();
    if (pnh_.param<bool>("confirm_settings",true))
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
        if(pnh_.param<bool>("gyrometer_calibration",true))
        {
            ROS_INFO("gyrometer calibration");
            serialPort.write("<cg>");    // gyrometer calibration
            rate.sleep();
        }
        if(pnh_.param<bool>("accelerometer_calibration",true))
        {
            ROS_INFO("accelerometer calibration");
            serialPort.write("<cas>");;    // accelerometer calibration
            rate.sleep();
        }
        if (pnh_.param<bool>("magnetometer_calibration", true))
        {
            ROS_INFO("magnetometer calibration");
            serialPort.write("<cmf>");;    // megnetormeter calibration
            rate.sleep();
        }
        rate.sleep();
        rate.sleep();
        ROS_INFO("set output format");
    }

    ROS_INFO("IMU Ready");

    imuSample = Eigen::MatrixXd::Zero(sampleNum,10);
    initSample = true;

    pubImu = nh_.advertise<sensor_msgs::Imu>(publish_topic_name,1);
}


void ebimu::publishData()
{
    if(serialPort.available())
    {
        std::string data;
        data = serialPort.read(1);
        if (data.c_str()[0] == 0x55)
        {
            data = serialPort.read(1);
            if(data.c_str()[0] == 0x55)
            {
                data = serialPort.read(26);
                int channel = data.c_str()[0];
                int id = data.c_str()[1];
                int16_t quat_z = ((uint8_t)data.c_str()[2] << 8 | (uint8_t)data.c_str()[3]);
                int16_t quat_y = ((uint8_t)data.c_str()[4] << 8 | (uint8_t)data.c_str()[5]);
                int16_t quat_x = ((uint8_t)data.c_str()[6] << 8 | (uint8_t)data.c_str()[7]);
                int16_t quat_w = ((uint8_t)data.c_str()[8] << 8 | (uint8_t)data.c_str()[9]);
                int16_t gyro_x = ((uint8_t)data.c_str()[10] << 8 | (uint8_t)data.c_str()[11]);
                int16_t gyro_y = ((uint8_t)data.c_str()[12] << 8 | (uint8_t)data.c_str()[13]);
                int16_t gyro_z = ((uint8_t)data.c_str()[14] << 8 | (uint8_t)data.c_str()[15]);
                int16_t accel_x = ((uint8_t)data.c_str()[16] << 8 | (uint8_t)data.c_str()[17]);
                int16_t accel_y = ((uint8_t)data.c_str()[18] << 8 | (uint8_t)data.c_str()[19]);
                int16_t accel_z = ((uint8_t)data.c_str()[20] << 8 | (uint8_t)data.c_str()[21]);
                int16_t battery_remain = ((uint8_t)data.c_str()[22] << 8 | (uint8_t)data.c_str()[23]);

                double f_quat_z = quat_z / 10000.0;
                double f_quat_y = quat_y / 10000.0;
                double f_quat_x = quat_x / 10000.0;
                double f_quat_w = quat_w / 10000.0;

                double f_gyro_z = gyro_z / 10.0 * DegreetoRadian;
                double f_gyro_y = gyro_y / 10.0 * DegreetoRadian;
                double f_gyro_x = gyro_x / 10.0 * DegreetoRadian;

                double f_accel_z = accel_z / 1000.0 * gtometerpersec;
                double f_accel_y = accel_y / 1000.0 * gtometerpersec;
                double f_accel_x = accel_x / 1000.0 * gtometerpersec;

                Eigen::MatrixXd data(1,10);
                data << f_quat_w , f_quat_x , f_quat_y , f_quat_z, f_gyro_x , f_gyro_y , f_gyro_z, f_accel_x , f_accel_y , f_accel_z;

                // screen
                std::cout.precision(4);
                std::cout
                        << std::endl << std::fixed
                        << "channel : " << channel << " "
                        << "id : " << id << std::endl
                        << "quat(wxyz)\t: "
                        <<  data(QUAT_W)<< "\t"
                        <<  data(QUAT_X)<< "\t"
                        <<  data(QUAT_Y)<< "\t"
                        <<  data(QUAT_Z)<< "\n"
                        << "gyro\t\t: "
                        <<  data(GYRO_X)<< "\t"
                        <<  data(GYRO_Y)<< "\t"
                        <<  data(GYRO_Z)<< "\n"
                        << "accel\t\t: "
                        <<  data(ACCEL_X)<< "\t"
                        <<  data(ACCEL_Y)<< "\t"
                        <<  data(ACCEL_Z)<< "\n"
                        << "battery\t\t: "<< battery_remain  << std::endl;


                msg.header.frame_id =  std::to_string(channel) + "-" + std::to_string(id);
                msg.header.stamp = ros::Time::now();
                msg.orientation.w = data(IDX::QUAT_W);
                msg.orientation.x = data(IDX::QUAT_X);
                msg.orientation.y = data(IDX::QUAT_Y);
                msg.orientation.z = data(IDX::QUAT_Z);
                msg.angular_velocity.x = data(IDX::GYRO_X);
                msg.angular_velocity.y = data(IDX::GYRO_Y);
                msg.angular_velocity.z = data(IDX::GYRO_Z);
                msg.linear_acceleration.x = data(IDX::ACCEL_X);
                msg.linear_acceleration.y = data(IDX::ACCEL_Y);
                msg.linear_acceleration.z = data(IDX::ACCEL_Z);

                if (initSample)
                {
                    std::cout << "init sample \n\n";
                    imuSample.block(0,0,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,f_quat_w);
                    imuSample.block(0,1,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,f_quat_x);
                    imuSample.block(0,2,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,f_quat_y);
                    imuSample.block(0,3,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,f_quat_z);

                    imuSample.block(0,4,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,f_gyro_x);
                    imuSample.block(0,5,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,f_gyro_y);
                    imuSample.block(0,6,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,f_gyro_z);

                    imuSample.block(0,7,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,f_accel_x);
                    imuSample.block(0,8,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,f_accel_y);
                    imuSample.block(0,9,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,f_accel_z);
                    std::cout << "init sample finish\n\n";
                    initSample = false;
                }
                getCovariance(imuSample,data,R);
                msg.orientation_covariance[0] = R(IDX::QUAT_X, IDX::QUAT_X);
                msg.orientation_covariance[4] = R(IDX::QUAT_Y, IDX::QUAT_Y);
                msg.orientation_covariance[8] = R(IDX::QUAT_Z, IDX::QUAT_Z);
                msg.angular_velocity_covariance[0] = R(IDX::GYRO_X, IDX::GYRO_X);
                msg.angular_velocity_covariance[4] = R(IDX::GYRO_Y, IDX::GYRO_Y);
                msg.angular_velocity_covariance[8] = R(IDX::GYRO_Z, IDX::GYRO_Z);
                msg.linear_acceleration_covariance[0] = R(IDX::ACCEL_X, IDX::ACCEL_X);
                msg.linear_acceleration_covariance[4] = R(IDX::ACCEL_Y, IDX::ACCEL_Y);
                msg.linear_acceleration_covariance[8] = R(IDX::ACCEL_Z, IDX::ACCEL_Z);
                pubImu.publish(msg);
            }
            else
            {
                std::cout << data;
            }
        }
        else
        {
            std::cout << data;
        }
    }
}
