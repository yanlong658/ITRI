#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "itri/process.h"
#include "itri/parameter.h"

#define epipolar

using namespace std;
using namespace cv;

#ifndef epipolar
//這邊分成兩部份,1是直接用topic接收影像資料,另一是直接傳入image當作test data
typedef std::pair<sensor_msgs::ImageConstPtr ,sensor_msgs::ImageConstPtr> CombinedData;
// Camera buffer
std::queue<sensor_msgs::ImageConstPtr> camera1Buf;
std::queue<sensor_msgs::ImageConstPtr> camera2Buf;
std::queue<CombinedData> measurements;
std::mutex m_buf, com_buf;
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
#endif
void computeYPR(Eigen::Matrix3d rotation_matrix)
{
    Eigen::Vector3d euler_angles_Epi = rotation_matrix.eulerAngles(2, 1, 0);
    euler_angles_Epi[0] = euler_angles_Epi[0]/M_PI*180.0;
    euler_angles_Epi[1] = euler_angles_Epi[1]/M_PI*180.0;
    euler_angles_Epi[2] = euler_angles_Epi[2]/M_PI*180.0;
    std::cout << "euler_angles_Epi Y P R = " << euler_angles_Epi.transpose() << std::endl;

}

cv::Mat q1to2(Eigen::Quaterniond qw_1 , Eigen::Quaterniond qw_2)
{
    cv::Mat R;
    Eigen::Matrix3d rotation_matrix;
    Eigen::Quaterniond q1_2;
    q1_2 = qw_2*qw_1.inverse();

    rotation_matrix = q1_2.matrix();

    std::cout<<"rotation_matrix: "<< rotation_matrix <<std::endl;

    computeYPR(rotation_matrix);
    cv::eigen2cv(rotation_matrix,R);
    std::cout<<"R : "<< R <<std::endl;
    return R;
}

Mat W2B(Mat& t,Eigen::Quaterniond q_)
{
    Eigen::Vector3d v1_;
    cv2eigen(t,v1_);
    v1_ = q_*v1_;
    t = (Mat_<double> (3,1) <<  v1_(0), v1_(1), v1_(2));

    std::cout<<"t: "<<t<<std::endl;
}

Point2f pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2f
    (
        ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0),
        ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1)
    );
}


void triangulation (const Mat& R, const Mat& t, vector< Point3d >& points )
{
    Mat T1 = (Mat_<float> (3,4) <<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0);
    Mat T2 = (Mat_<float> (3,4) <<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    );

    //1280 1024
    Mat K1 = ( Mat_<double> ( 3,3 ) << 1450.0, 0.0, 663.4,
                                      0.0, 1450.0, 540.0,
                                      0.0, 0.0, 1.0 );

    Mat K2 = ( Mat_<double> ( 3,3 ) << 1340.0,0.0, 639.1,
                                      0.0, 1337.6, 516.2,
                                      0.0, 0.0, 1.0 );

    vector<Point2f> pts_1, pts_2,pts_3, pts_4;

    Point2d x1_1(549.0,697.0);
    Point2d x2_1(619.0,651.0);
    Point2d x3_1(543.0,703.0);
    Point2d x4_1(608.0,650.0);

    pts_1.push_back ( pixel2cam(x1_1,K1) );
    pts_2.push_back ( pixel2cam(x2_1,K2) );

    pts_3.push_back ( pixel2cam(x3_1,K1) );
    pts_4.push_back ( pixel2cam(x4_1,K2) );

    Mat pts_4d_1,pts_4d_2;
    cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d_1 );
    cv::triangulatePoints( T1, T2, pts_3, pts_4, pts_4d_2 );

    Mat x = pts_4d_1.col(0);
    x /= x.at<float>(3,0);
    std::cout<<"x : "<<x<<std::endl;
    Point3d p1 (
        x.at<float>(0,0),
        x.at<float>(1,0),
        x.at<float>(2,0)
    );
    std::cout<<"P : "<<p1<<std::endl;
    points.push_back( p1 );


    Mat y = pts_4d_2.col(0);
    y /= y.at<float>(3,0);
    std::cout<<"y : "<<y<<std::endl;
    Point3d p2 (
        y.at<float>(0,0),
        y.at<float>(1,0),
        y.at<float>(2,0)
    );
    std::cout<<"P : "<<p2<<std::endl;
    points.push_back( p2 );

    std::cout<<"Norm : "<<std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z))<<std::endl;
}
#ifndef epipolar
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
#endif

#ifndef epipolar
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
#endif
int main(int argc, char** argv)
{
    ros::init(argc, argv, "initial");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    cv::Mat R,t;

    // 1->2
    Eigen::Quaterniond qw_1(-0.864, -0.133,-0.299, 0.380);
    Eigen::Quaterniond qw_2(-0.919, 0.038,-0.346, -0.180);

    // t要在camera 2
    t = (cv::Mat_<double>(3,1) <<-44.9, 1692.0, 35.0);

    R = q1to2(qw_1,qw_2);
    W2B(t,qw_2);

    //-- 三角化
    vector<Point3d> points;
    triangulation( R, t, points);

#ifndef epipolar
    // 底下是計算epipolar
    imageL = cv::imread(path1,1);
    imageR = cv::imread(path2,1);

    // put your image
    ros::Subscriber image1_sub = nh.subscribe<sensor_msgs::Image>(path_topic1, 100, camera1);
    ros::Subscriber image2_sub = nh.subscribe<sensor_msgs::Image>(path_topic2, 100, camera2);

    std::thread measurement{command};
#endif

    ros::spin();
    return 0;
}

