#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <bitset>

#include "selfcar_lib/covariance.h"
#include "selfcar_lib/erp42.h"


int main(int argc, char** argv)
{
    ros::init(argc,argv, "test_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ERP42 erp(nh,pnh);
    ros::Rate r(20.0);
    while(ros::ok())
    {
        std::cout << erp.getAorM() << std::endl;
        std::cout << erp.getEstop() << std::endl;
        std::cout << erp.getGear() << std::endl;
        std::cout << erp.getBreak() << std::endl;
        std::cout << erp.getVelocity() << std::endl;
        std::cout << erp.getSteer() << std::endl;
        std::cout << erp.getEncoder() << std::endl << std::endl;
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
