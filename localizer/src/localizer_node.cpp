#include <ros/ros.h>

#include "localizer/localizer.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"/localizer");
    Localizer l;
    ros::spin();
    return 0;
}
