#ifndef ERP42_H
#define ERP42_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

class ERP42
{
public:

    ERP42(ros::NodeHandle& node, ros::NodeHandle& prv_node);
    bool isStop();
    bool isERPavailable();
    const std::string getAorM();
    const std::string getEstop();
    const std::string getGear();
    const int getBreak();
    const double getVelocity();
    const int getSteer();
    const int getEncoder();
    const std::string getState();
private:
    void aormCallback(const std_msgs::String::ConstPtr& msg);
    void estopCallback(const std_msgs::String::ConstPtr& msg);
    void gearCallback(const std_msgs::String::ConstPtr& msg);
    void breakCallback(const std_msgs::Int8::ConstPtr& msg);
    void velCallback(const std_msgs::Int16::ConstPtr& msg);
    void steerCallback(const std_msgs::Int16::ConstPtr& msg);
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

    ros::Subscriber subAorM;
    ros::Subscriber subEstop;
    ros::Subscriber subGear;
    ros::Subscriber subBreak;
    ros::Subscriber subVel;
    ros::Subscriber subSteer;
    ros::Subscriber subEnc;

    // data
    std_msgs::String aorm;
    std_msgs::String estop;
    std_msgs::String gear;
    std_msgs::Int8 brk;
    std_msgs::Int16 vel;
    std_msgs::Int16 steer;
    std_msgs::Int32 enc;
    int state;

    // flag
    bool aormEnable;
    bool estopEnable;
    bool gearEnable;
    bool breakEnable;
    bool velEnable;
    bool steerEnable;
    bool encEbable;
};

#endif // ERP42_H
