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

class imageProcess
{
  public:
    imageProcess();

    cv::Point2f distortion(cv::Point2f Point);

    cv::Point2f pixel2cam(const cv::Point2f &p, const cv::Mat &K);

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
    // opencv solvepnp 相機內參和distortion是使用float,其他都是double
    cv::Mat k_g = (cv::Mat_<float>(3,3) << 829.741843, 0.0, 328.720164, 0.0, 830.451921, 238.134520, 0.0, 0.0, 1.0);
    cv::Mat k_b = (cv::Mat_<float>(3,3) << 866.356440, 0.0, 326.529608, 0.0, 862.655564, 263.425809, 0.0, 0.0, 1.0);
    cv::Mat k_t = (cv::Mat_<float>(3,3) << 866.356440, 0.0, 326.529608, 0.0, 862.655564, 263.425809, 0.0, 0.0, 1.0);

    //distortion coefficient
    cv::Mat dis_coff = (cv::Mat_<float>(1,5) << -0.347793, 0.107340, 0.000000, 0.000722, -0.001039);

    float k1 = -0.347793;
    float k2 =  0.107340;
    float k3 =  0.000000;
    float p1 =  0.000722;
    float p2 = -0.001039;

    // test data
    std::vector<cv::Point2f> cameraData1er;
    std::vector<cv::Point2f> cameraData2er;
    cv::Point2f cameraData1;
    cv::Point2f cameraData2;

    int count;
};
#endif // IMAGEPROCESS_H
