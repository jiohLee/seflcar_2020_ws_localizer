#ifndef ERP42_H
#define ERP42_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

class ERP42
{
public:

    ERP42(ros::NodeHandle& node, ros::NodeHandle& prv_node);
    bool isStop();
    bool isERPavailable();
    const double getVelocity();
    const int getEncoderValue();
    const int getState();
private:
    void velCallback(const std_msgs::Int16::ConstPtr& msg);
    void encCallback(const std_msgs::Int32::ConstPtr& msg);

    enum IDX
    {
        STOP = 0,
        FORWARD = 1,
        BACKWARD = 2,
        GO = 3
    };

    // ros
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber subVel;
    ros::Subscriber subEnc;

    // data
    std_msgs::Int16 vel;
    std_msgs::Int32 enc;
    int state;

    // flag
    bool velEnable;
    bool encEbable;
};

#endif // ERP42_H
