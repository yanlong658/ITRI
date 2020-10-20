#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "itri/process.h"
#include "itri/parameter.h"

// 相機校正
typedef std::pair<sensor_msgs::ImageConstPtr ,sensor_msgs::ImageConstPtr> CombinedData;

std::queue<sensor_msgs::ImageConstPtr> camera1Buf;
std::queue<sensor_msgs::ImageConstPtr> camera2Buf;
std::queue<CombinedData> measurements;

std::mutex m_buf, com_buf;

imageProcess imageprocess;

//test
cv::Mat imageL,imageR;

void camera1(const sensor_msgs::ImageConstPtr &Image_msg)
{
  m_buf.lock();
  camera1Buf.push(Image_msg);
  m_buf.unlock();
}

void camera2(const sensor_msgs::ImageConstPtr &Image_msg)
{
  m_buf.lock();
  camera2Buf.push(Image_msg);
  m_buf.unlock();
}

CombinedData getMeasurement()
{
  CombinedData measurement;
  if(!camera1Buf.empty() && !camera2Buf.empty())
  {
    measurement = std::make_pair(camera1Buf.back(),camera2Buf.back());
  }
  return measurement;
}

//pixel frame translate to camera frame
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K)
{
  return cv::Point2d
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

void process()
{
  cv_bridge::CvImageConstPtr ptr1,ptr2;
  cv::Mat image1 , image2;
/*
  image1 = imageprocess.getImageFromMsg(measurements.back().first, ptr1);
  image2 = imageprocess.getImageFromMsg(measurements.back().second, ptr2);
*/
  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
  std::vector<cv::DMatch> matches;

  // match feature
  imageprocess.find_feature_matches(imageL, imageR, keypoints_1, keypoints_2, matches);
  std::cout<<"How much pairs : "<<matches.size()<<std::endl;

  // estimate the transformation(r , t) from 1 to 2
  cv::Mat R,t;
  imageprocess.pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

  std::cout<<"pose_estimation_2d2d R : "<<R<<std::endl;
  std::cout<<"pose_estimation_2d2d t : "<<t<<std::endl;

  //traingulation
  std::vector<cv::Point3d> points;
  imageprocess.triangulation(keypoints_1, keypoints_2, matches, R, t, points);

/*
  //verify the re-projected error in 3d-2d
  for (int i = 0; i < matches.size(); i++)
  {
      // first photo
      cv::Point2d pt1_cam = pixel2cam(keypoints_1[matches[i].queryIdx].pt, k_b);
      cv::Point2d pt1_cam_3d(points[i].x/points[i].z, points[i].y/points[i].z);

      cv::circle(imageL,keypoints_1[i].pt,3,cv::Scalar(12,12,12),-1);

      // second photo
      cv::Point2d pt2_cam = pixel2cam(keypoints_2[matches[i].trainIdx].pt, k_g );
      cv::Mat pt2_trans = R * (cv::Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
      pt2_trans /= pt2_trans.at<double>(2,0);
      cv::circle(imageR,keypoints_2[i].pt,3,cv::Scalar(0,255,0),-1);

  }
*/

// solve the pnp to enforce the R and t

  std::vector<cv::Point3f> pts_3d;
  std::vector<cv::Point2f> pts_2d;

  imageprocess.Pnp(matches, points, pts_2d, keypoints_1, keypoints_2, R, t ,imageL);

  std::cout<<"pnp R : "<<R<<std::endl;
  std::cout<<"pnp t : "<<t<<std::endl;

  points.clear();
  std::cout<<"points.size : "<<points.size()<<std::endl;
  imageprocess.triangulation(keypoints_1, keypoints_2, matches, R, t, points);

  cv::Mat img_RR_matches;
  // drawe the matched photo
  cv::drawMatches(imageL,keypoints_1,imageR,keypoints_2,matches,img_RR_matches, cv::Scalar(0, 255, 0));
  cv::imshow("match",img_RR_matches);

  cv::imshow("L" , imageL);
  cv::imshow("R" , imageR);
  cv::waitKey();
}

// 只要計算一次就好了
void command()
{
  while(ros::ok)
  {
    char c = getchar();
    if(c == 's')
    {
      com_buf.lock();
      measurements.push(getMeasurement());
      com_buf.unlock();
      process();
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "initial");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  readParameters(nh);

  imageL = cv::imread(path1,1);
  imageR = cv::imread(path2,1);

  // put your image
  ros::Subscriber image1_sub = nh.subscribe<sensor_msgs::Image>(path_topic1, 100, camera1);
  ros::Subscriber image2_sub = nh.subscribe<sensor_msgs::Image>(path_topic2, 100, camera2);

  std::thread measurement{command};
  ros::spin();
  return 0;

}

