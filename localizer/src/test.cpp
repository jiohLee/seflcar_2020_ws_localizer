#include <ros/ros.h>

#include "selfcar_lib/covariance.h"


int main(int argc, char** argv)
{
    ros::init(argc,argv, "test_node");
    ros::NodeHandle nh;

    Eigen::MatrixXd test(2,2);
    test << 100, 100, 200, 300;
    Eigen::MatrixXd avg(2,2);
    getAverage(test, avg);
    std::cout << avg << std::endl;

    return 0;
}
