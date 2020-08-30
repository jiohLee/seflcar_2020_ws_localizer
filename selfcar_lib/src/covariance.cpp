#ifndef COVARIANCE_H
#define COVARIANCE_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <iostream>
#include <ros/ros.h>

void updateSample(Eigen::MatrixXd &sample, Eigen::MatrixXd &data)
{
    if (data.cols() != sample.cols() || data.rows() != 1)
    {
        ROS_WARN("invalid data for sample matrix");
        return;
    }
    Eigen::MatrixXd temp = sample.block(1,0,sample.rows() - 1, sample.cols());
    sample.block(0,0,sample.rows() - 1, sample.cols()) = temp;
    sample.block(sample.rows()- 1, 0, 1, sample.cols()) = data;
}

void getCovariance(Eigen::MatrixXd &sample, Eigen::MatrixXd &data, Eigen::MatrixXd &R)
{
    if (data.cols() != sample.cols() || data.rows() != 1)
    {
        ROS_WARN("invalid data for sample matrix");
        return;
    }

    updateSample(sample, data);
    Eigen::MatrixXd deviation = sample.rowwise() - sample.colwise().mean();
    R = (deviation.transpose() * deviation) / static_cast<double>(sample.rows()); // covariance matrix
}

void getCovariance(Eigen::MatrixXd &sample, Eigen::MatrixXd &R)
{
    Eigen::MatrixXd deviation = sample.rowwise() - sample.colwise().mean();
    R = (deviation.transpose() * deviation) / static_cast<double>(sample.rows()); // covariance matrix
}

void getAverage(Eigen::MatrixXd &sample, Eigen::MatrixXd &avg)
{
    avg = sample.colwise().mean();
}

#endif // COVARIANCE_H




