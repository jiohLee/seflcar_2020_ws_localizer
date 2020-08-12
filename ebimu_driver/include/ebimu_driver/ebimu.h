#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Dense>

class ebimu
{
public:
    ebimu();
    void publishData();
private:

    enum IDX
    {
        QUAT_W = 0,
        QUAT_X = 1,
        QUAT_Y = 2,
        QUAT_Z = 3,
        GYRO_X = 4,
        GYRO_Y = 5,
        GYRO_Z = 6,
        ACCEL_X = 7,
        ACCEL_Y = 8,
        ACCEL_Z = 9
    };

    // ros
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher pubImu;
    sensor_msgs::Imu msg;

    // data
    bool initSample;
    int sampleNum;
    Eigen::MatrixXd imuSample;  // quat(w x y z), gyro(x,y,z), accelo(x,y,z)
    Eigen::MatrixXd R;          // Covariance Matrix of all data;

    // serial
    serial::Serial serialPort;

    // constant
    static const float_t DegreetoRadian;
    static const float_t gtometerpersec;
};
