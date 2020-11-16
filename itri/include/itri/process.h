#ifndef PROCESS_H
#define PROCESS_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "itri/parameter.h"

class imageProcess
{
  public:
    imageProcess();
    cv::Point2f distortion_9(cv::Point2f Point);
    cv::Point2f distortion_10(cv::Point2f Point);

    cv::Point2f pixel2cam_9(const cv::Point2f &p, const cv::Mat &K);
    cv::Point2f pixel2cam_10(const cv::Point2f &p, const cv::Mat &K);

    cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr img_msg , cv_bridge::CvImageConstPtr &ptr);

    void find_feature_matches(cv::Mat &image1 ,cv::Mat &image2,std::vector<cv::KeyPoint> &keypoints_1 ,
                              std::vector<cv::KeyPoint> &keypoints_2,std::vector<cv::DMatch> &good_matches);

    void pose_estimation_2d2d(std::vector<cv::KeyPoint> &keypoint_1 , std::vector<cv::KeyPoint> &keypoint_2,
                              std::vector<cv::DMatch> &matches, cv::Mat &R,cv::Mat &t);

    void triangulation(const std::vector<cv::KeyPoint> &keypoint_1, const std::vector<cv::KeyPoint> &keypoint_2,
                       const std::vector<cv::DMatch> &matches, const cv::Mat R , const cv::Mat t, std::vector<cv::Point3f> &points);

    void Pnp(std::vector<cv::DMatch> &matches, std::vector<cv::Point3f> &pts_3d, std::vector<cv::Point2f> &pts_2d
             ,std::vector<cv::KeyPoint> &keypoints_1, std::vector<cv::KeyPoint> &keypoints_2,cv::Mat &R, cv::Mat &t, cv::Mat);

  private:
    cv::Mat k_g = (cv::Mat_<float>(3,3) << 1199.7, 0.0, 673.2, 0.0, 1201.5, 511.1, 0.0, 0.0, 1.0);
    cv::Mat k_b = (cv::Mat_<float>(3,3) << 1197.1, 0.0, 629.3, 0.0, 1198.1, 497.2, 0.0, 0.0, 1.0);
    cv::Mat k_t = (cv::Mat_<float>(3,3) << 1199.7, 0.0, 673.2, 0.0, 1201.5, 511.1, 0.0, 0.0, 1.0);

    //distortion coefficient
    cv::Mat dis_coff = (cv::Mat_<float>(1,5) << -0.138, 0.241, -0.3443, 0.0, 0.0);

    float k1_9 = -0.138;
    float k2_9 = 0.241;
    float k3_9 = -0.3443;
    float p1_9 = 0.0;
    float p2_9 = 0.0;

    float k1_10 = -0.1357;
    float k2_10 = 0.163;
    float k3_10 = -0.5696;
    float p1_10 = 0.0;
    float p2_10 = 0.0;

    // test data
    std::vector<cv::Point2f> cameraData1er;
    std::vector<cv::Point2f> cameraData2er;
    cv::Point2f cameraData1;
    cv::Point2f cameraData2;

    int count;
};
#endif // IMAGEPROCESS_H
