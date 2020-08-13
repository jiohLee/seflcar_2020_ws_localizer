#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <bitset>

#include "selfcar_lib/covariance.h"


int main(int argc, char** argv)
{
    ros::init(argc,argv, "test_node");
    ros::NodeHandle nh;

    uint8_t a = 0xFF; // 0b 0011 0001
    uint8_t b = 0xFF; // 0b 1011 0010
    int16_t c = a << 8 | b ;

    ROS_INFO("c : %d, %X %X", c, a, b);

    ros::Publisher pub1 = nh.advertise<std_msgs::Int32>("/testnode/1",1);
    ros::Publisher pub2 = nh.advertise<std_msgs::Int32>("/testnode/2",1);
    ros::Publisher pub3 = nh.advertise<std_msgs::Int32>("/testnode/3",1);
    ros::Publisher pub4 = nh.advertise<std_msgs::Int32>("/testnode/4",1);

    std_msgs::Int32 msg1;
    msg1.data = 1;
    std_msgs::Int32 msg2;
    msg2.data = 10;
    std_msgs::Int32 msg3;
    msg3.data = 100;
    std_msgs::Int32 msg4;
    msg4.data = 1000;

    ros::Rate r(10);
    while(ros::ok())
    {
        pub1.publish(msg1);
        pub2.publish(msg2);
        pub3.publish(msg3);
        pub4.publish(msg4);
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
