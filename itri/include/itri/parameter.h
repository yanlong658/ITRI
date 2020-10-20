#pragma once
#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>


extern std::string path1;
extern std::string path2;

extern std::string path_topic1;
extern std::string path_topic2;


extern cv::Mat r_12;
extern cv::Mat t_12;

void readParameters(ros::NodeHandle &n);
#endif
