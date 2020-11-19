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

// CombineData receive the topic data
typedef std::pair<sensor_msgs::ImageConstPtr ,sensor_msgs::ImageConstPtr> CombinedData;

// Camera buffer
std::queue<sensor_msgs::ImageConstPtr> camera1Buf;
std::queue<sensor_msgs::ImageConstPtr> camera2Buf;
std::queue<CombinedData> measurements;

std::mutex m_buf, com_buf;
imageProcess imageprocess;

// camera instrinsic
cv::Mat k_g = (cv::Mat_<float>(3,3) << 1199.7, 0.0, 673.2, 0.0, 1201.5, 511.1, 0.0, 0.0, 1.0);
cv::Mat k_b = (cv::Mat_<float>(3,3) << 1197.1, 0.0, 629.3, 0.0, 1198.1, 497.2, 0.0, 0.0, 1.0);

// test data
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

// get the right and left photo
CombinedData getMeasurement()
{
  CombinedData measurement;
  if(!camera1Buf.empty() && !camera2Buf.empty())
  {
    measurement = std::make_pair(camera1Buf.back(),camera2Buf.back());
  }
  return measurement;
}

cv::Mat W2B(cv::Mat& v1)
{
    // translation to camera frame
    Eigen::Vector3d t_(20.0,-10.0,-50.0);
    Eigen::Matrix3d rotation_matrixW2c;
    Eigen::Quaterniond qw2c;
    rotation_matrixW2c <<   0,-1,0,
                            0,0,-1,
                            1,0,0;

    qw2c = rotation_matrixW2c;
    Eigen::Vector3d v1_;
    Eigen::Quaterniond q_(-0.4811, -0.1882,-0.2972, -0.8029);
    cv2eigen(v1,v1_);
    //v1_ = qw2c*q_*v1_;
    v1_ = q_*v1_+t_;
    v1 = (cv::Mat_<double> (3,1) <<  v1_(0), v1_(1), v1_(2));

    std::cout<<"v1: "<<v1<<std::endl;
}

// process the data
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

/*
  std::cout<<"pose_estimation_2d2d R : "<<R<<std::endl;
  std::cout<<"pose_estimation_2d2d t : "<<t<<std::endl;
*/

  std::vector<cv::Point3f> points;
  // Extrinsic parameter camera 1-> camera 2
  R = (cv::Mat_<double>(3,3)<< 0.329991, -0.744275,  0.580378,
                               0.735088,  0.588367,   0.33656,
                              -0.591971,  0.315724,  0.741461);

  t = (cv::Mat_<double>(3,1) <<265.0, 1406.0, -0.01);
  W2B(t);
  // traingulate to find the point position
  imageprocess.triangulation(keypoints_1, keypoints_2, matches, R, t, points);



  // solve the pnp to enforce the R and t
  std::vector<cv::Point2f> pts_2d;

  //imageprocess.Pnp(matches, points, pts_2d, keypoints_1, keypoints_2, R, t ,imageL);

  std::cout<<"pnp R : "<<R<<std::endl;
  std::cout<<"pnp t : "<<t<<std::endl;

  points.clear();
  std::cout<<"points.size : "<<points.size()<<std::endl;
  imageprocess.triangulation(keypoints_1, keypoints_2, matches, R, t, points);

  // set the tansformation to all global parameter
  r_12 = R;
  t_12 = t;

  cv::Mat img_RR_matches;
  // drawe the matched photo
  cv::drawMatches(imageL,keypoints_1,imageR,keypoints_2,matches,img_RR_matches, cv::Scalar(0, 255, 0));

  cv::imwrite("/home/yanlong658/Desktop/error.jpg", img_RR_matches);
  cv::Mat rs;
  cv::resize(img_RR_matches,rs,cv::Size(img_RR_matches.cols/1.5,img_RR_matches.rows/1.5),0,0,cv::INTER_LINEAR);
  cv::imshow("match",rs);

  //cv::imshow("L" , imageL);
  //cv::imshow("R" , imageR);
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

