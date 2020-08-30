#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <novatel_gps_msgs/NovatelVelocity.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

#include <string>
#include <vector>
#include <math.h>
#include <iomanip>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

class GNSSLocalizer
{
public:
    enum IDX
    {
        X = 0,
        Y = 1,
        YAW = 2,
        V = 3
    };
    GNSSLocalizer(ros::NodeHandle& node, ros::NodeHandle& prv_node);
    const Eigen::MatrixXd getStateMatrix();
    const Eigen::MatrixXd getStateCovMatrix();
    bool isGPSavailable();
    bool isStop();
private:
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void bestvelCallback(const novatel_gps_msgs::NovatelVelocity::ConstPtr& msg);

    // ros
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    ros::Subscriber subGps;
    ros::Subscriber subVel;

    // data
    sensor_msgs::NavSatFix gps;
    novatel_gps_msgs::NovatelVelocity vel;
    Eigen::MatrixXd gpsSample;
    Eigen::MatrixXd X_; // x y theta velocity
    Eigen::MatrixXd R_; // covariance;
    int sampleNum;

    // constant
    static const double gpsUpdateRate;
    static const int dataNum;

    // flag
    bool bInitgps;
    bool bInitgpsSample;
    bool bInitvelSample;
    bool bGPSavailable;
};

