#include <ros/ros.h>

#include "ebimu_driver/ebimu.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"/ebimu_driver");
    ebimu e;
    while(ros::ok())
    {
        e.publishData();
        ros::spinOnce();
    }
    return 0;
}
