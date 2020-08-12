#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <novatel_gps_msgs/NovatelVelocity.h>
#include <novatel_gps_msgs/NovatelPosition.h>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <vector>
#include <math.h>
#include <iomanip>

#include "selfcar_lib//erp42.h"
#include "selfcar_lib/kalman_filter.h"
#include "selfcar_lib/navsat_conversions.h"
#include "selfcar_lib/covariance.h"

//typedef struct Kalman
//{
//    geometry_msgs::PoseWithCovariance pos;
//    long double velocity_estimate;
//}Kalman;

class Localizer
{
public:
    Localizer();
private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void bestvelCallback(const novatel_gps_msgs::NovatelVelocity::ConstPtr& msg);
    void bestposCallback(const novatel_gps_msgs::NovatelPosition::ConstPtr& msg);
    void filter();
    void predict();
    void updateWithGate();

    // ros
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber subIMU;
    ros::Subscriber subGPS;
    ros::Subscriber subBestVel;
    ros::Subscriber subBestPos;

    ros::Publisher pubPose;
    ros::Publisher pubMarker;
    ros::Publisher pubMarker_filtered;

    // message
    visualization_msgs::Marker marker_filtered;

    // imu - data
    sensor_msgs::Imu imu;
    double localHeading;
    double headings;
    tf2::Quaternion qYawBias;

    // imu - param
    int calibrationFrameCount;
    int frameCount;

    // imu - flag
    bool bCalibrationDone;

    // gps - data
    sensor_msgs::NavSatFix gps;
    novatel_gps_msgs::NovatelVelocity bestvel;
    novatel_gps_msgs::NovatelPosition bestpos;
    Eigen::MatrixXd gpsSample;
    Eigen::MatrixXd gpsData; // x y theta velocity
    Eigen::MatrixXd gpsCov; // covariance;
    int sampleNum;

    // gps - flag
    bool bInitgps;
    bool bInitgpsSample;
    bool bInitvelSample;
    bool isGPSstop;
    bool isGPSavailable;

    // gps - param
    double utmoffsetX; // meter
    double utmoffsetY; // meter

    // ERP42
    ERP42 erp;

    // KalmanFilter
    KalmanFilter kf_;
    int dim_x = 4;
    Eigen::MatrixXd X_prev;
    Eigen::MatrixXd P_prev;
    enum IDX
    {
        X = 0,
        Y = 1,
        YAW = 2,
        V = 3
    };
    bool bKalmanInit;

    // visualizer
    void visualizerCallback();
    int markerId;
    int markerId_filtered;

    // constants
    static const double imuUpdateRate;    // 0.016 second
    static const double gpsUpdateRate;    // 0.016 second
};
