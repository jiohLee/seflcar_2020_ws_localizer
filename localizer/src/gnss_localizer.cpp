#include "localizer/gnss_localizer.h"
#include "selfcar_lib/navsat_conversions.h"
#include "selfcar_lib/covariance.h"

const double GNSSLocalizer::gpsUpdateRate = 0.05;
const int GNSSLocalizer::dataNum = 4;

GNSSLocalizer::GNSSLocalizer(ros::NodeHandle& node, ros::NodeHandle& prv_node)
    : nh_(node)
    , pnh_(prv_node)
{
    std::string gps_subscribe_topic_name;
    std::string gps_vel_subscribe_topic_name;
    bool gps_velocity_estimate;

    pnh_.param<std::string>("gps_subscribe_topic_name",gps_subscribe_topic_name,"/novatel_fix");
    pnh_.param<std::string>("gps_vel_subscribe_topic_name",gps_vel_subscribe_topic_name,"/bestvel");
    pnh_.param<int>("covariance_sample_num", sampleNum, 3);
    pnh_.param<bool>("gps_velocity_estimate",gps_velocity_estimate,true);

    subGps = nh_.subscribe(gps_subscribe_topic_name,1, &GNSSLocalizer::gpsCallback,this );
    if (gps_velocity_estimate)
    {
        subVel = nh_.subscribe(gps_vel_subscribe_topic_name,1, &GNSSLocalizer::bestvelCallback,this );
    }

    bInitgps = true;
    bInitgpsSample = false;
    bInitvelSample = false;
    bGPSavailable = false;

    gpsSample = Eigen::MatrixXd::Zero(sampleNum,dataNum);
    X_ = Eigen::MatrixXd::Zero(dataNum,1);
    R_ = Eigen::MatrixXd::Zero(dataNum,dataNum);
}

const Eigen::MatrixXd GNSSLocalizer::getStateMatrix()
{
    return X_;
}

const Eigen::MatrixXd GNSSLocalizer::getStateCovMatrix()
{
    return R_;
}

bool GNSSLocalizer::isGPSavailable()
{
    bool result = (!bInitgps && !bInitgpsSample) && bGPSavailable;
    bGPSavailable = false;
    return result;
}

bool GNSSLocalizer::isStop()
{
    if (X_(IDX::V) > 0.6)
    {
        return false;
    }
    return true;
}

void GNSSLocalizer::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    gps = *msg;

    double utm_x_meas = 0;
    double utm_y_meas = 0;
    std::string utm_zone;
    RobotLocalization::NavsatConversions::LLtoUTM(gps.latitude, gps.longitude,utm_y_meas, utm_x_meas, utm_zone);

    if(bInitgps)
    {
        X_(IDX::X) = utm_x_meas;
        X_(IDX::Y) = utm_y_meas;
        X_(IDX::YAW) = 0;
        X_(IDX::V) = 0;
        bInitgpsSample = true;
        bInitvelSample = true;
        bInitgps = false;
        return;
    }

    double theta = std::atan2(utm_y_meas - X_(IDX::Y), utm_x_meas - X_(IDX::X));
    X_(IDX::X) = utm_x_meas;
    X_(IDX::Y) = utm_y_meas;
    X_(IDX::YAW) = theta;

    if(bInitgpsSample)
    {
        gpsSample.block(0,IDX::X,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,X_(IDX::X));
        gpsSample.block(0,IDX::Y,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,X_(IDX::Y));
        gpsSample.block(0,IDX::YAW,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,X_(IDX::YAW));
        bInitgpsSample = false;
    }

    Eigen::MatrixXd data = X_.transpose();
    getCovariance(gpsSample, data, R_);
    bGPSavailable = true;
}

void GNSSLocalizer::bestvelCallback(const novatel_gps_msgs::NovatelVelocity::ConstPtr &msg)
{
    vel = *msg;
    X_(IDX::V) = vel.horizontal_speed;
    if(bInitvelSample)
    {
        gpsSample.block(0,3,sampleNum,1) = Eigen::MatrixXd::Constant(sampleNum,1,X_(IDX::V));
        bInitvelSample = false;
    }
}
