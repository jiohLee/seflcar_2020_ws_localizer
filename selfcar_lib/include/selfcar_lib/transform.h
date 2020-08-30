#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <iostream>

Eigen::Quaternionf toQuaternion(double roll, double pitch, double yaw);

static void toEulerAngle(const Eigen::Quaternionf& q, double& roll, double& pitch, double& yaw);

#endif // TRANSFORM_H
