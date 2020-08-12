#ifndef COVARIANCE_H
#define COVARIANCE_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <iostream>
#include <ros/ros.h>

void updateSample(Eigen::MatrixXd &sample, Eigen::MatrixXd &data);
void getCovariance(Eigen::MatrixXd &sample, Eigen::MatrixXd &data, Eigen::MatrixXd &R);
void getAverage(Eigen::MatrixXd& sample, Eigen::MatrixXd& avg);

#endif // COVARIANCE_H
