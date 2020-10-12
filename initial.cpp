#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
// features2d ORB,AKAZE
#include <opencv2/features2d.hpp>
// xfeatures2d SIFT,SURF
//#include <opencv2/sfm/triangulation.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

typedef std::pair<sensor_msgs::ImageConstPtr ,sensor_msgs::ImageConstPtr> CombinedData;

std::queue<sensor_msgs::ImageConstPtr> camera1Buf;
std::queue<sensor_msgs::ImageConstPtr> camera2Buf;
std::queue<CombinedData> measurements;
cv::Point2d principal_point ( 325.1, 249.7 );
double focal_length = 521;

// 416.811266, 0, 316.940768, 0, 417.101651, 251.243429,  0, 0, 1
// 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1
// k_l ,k_r 內參
cv::Mat k_g = (cv::Mat_<double>(3,3) << 829.7418430781596, 0, 328.7201641340541, 0, 830.4519219378317, 238.1345206129469, 0, 0, 1);
cv::Mat k_b = (cv::Mat_<double>(3,3) << 866.3564402390112, 0, 326.529608545326, 0, 862.6555640703662, 263.4258095061218, 0, 0, 1);

//test
cv::Mat imageL,imageR;

int frame_count =0;
std::mutex m_buf, com_buf;

// test center position of reflected ball
cv::Point2f cameraData1(195.0,305.0);
cv::Point2f cameraData2(376.0,308.0);

std::vector<cv::Point2f> cameraData1er;
std::vector<cv::Point2f> cameraData2er;

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
cv::Point2f pixel2cam(const cv::Point2d &p, const cv::Mat &K)
{
  return cv::Point2f
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

// tranglate the point
void triangulation(const std::vector<cv::KeyPoint> &keypoint_1, const std::vector<cv::KeyPoint> &keypoint_2,
                   const std::vector<cv::DMatch> &matches, const cv::Mat &R , const cv::Mat &t, std::vector<cv::Point3d> &points)
{
  cv::Mat T1 = (cv::Mat_<double>(3,4) <<
          1.0,0.0,0.0,0.0,
          0.0,1.0,0.0,0.0,
          0.0,0.0,1.0,0.0);

  cv::Mat T2 = (cv::Mat_<double>(3,4) <<
        R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0)/5.0,
        R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0)/5.0,
        R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0)/5.0);

  std::vector<cv::Point2d> pts_1,pts_2;

  for(cv::DMatch m:matches)
  {
    pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt,k_b));
    pts_2.push_back(pixel2cam(keypoint_2[m.queryIdx].pt,k_g));
  }

  cameraData1er.push_back(pixel2cam(cameraData1,k_b));
  cameraData2er.push_back(pixel2cam(cameraData2,k_g));

  cv::Mat pts_4d;
  cv::triangulatePoints(T1,T2,cameraData1er,cameraData2er,pts_4d);

  for(int i =0;i<pts_4d.cols;i++)
  {
    cv::Mat x = pts_4d.col(i);
    std::cout<<"x : "<<x<<std::endl;
    x /= x.at<float>(3,0);
    cv::Point3d P(x.at<float>(0,0), x.at<float>(1,0),x.at<float>(2,0));
    std::cout<<"P : "<<P<<std::endl;
    points.push_back(P);
  }
}

// estimator the R and t
void pose_estimation_2d2d(std::vector<cv::KeyPoint> &keypoint_1 , std::vector<cv::KeyPoint> &keypoint_2, std::vector<cv::DMatch> &matches, cv::Mat &R,cv::Mat &t)
{
  std::vector<cv::Point2f> points1;
  std::vector<cv::Point2f> points2;

  for(int i =0; i<(int) matches.size();i++)
  {
    points1.push_back(keypoint_1[matches[i].queryIdx].pt);
    points2.push_back(keypoint_2[matches[i].trainIdx].pt);
  }

  // 計算本質矩陣
  cv::Mat essential_matrix;
  // CV_FM_RANSAC for the RANSAC algorithm. N≥8
  // CV_FM_LMEDS for the LMedS algorithm. N≥8
  // essential_matrix = cv::findEssentialMat(points1,points2,k,CV_FM_RANSAC);
  essential_matrix = cv::findEssentialMat(points1,points2, k_b,CV_FM_RANSAC);
  std::cout<<"essential matrix : "<<essential_matrix<<std::endl;

  // cv::recoverPose(essential_matrix, points1,points2,k_b,R, t);
  cv::recoverPose(essential_matrix, points1,points2,k_b,R, t);
}

// find match points
void find_feature_matches(cv::Mat &image1 ,cv::Mat &image2,std::vector<cv::KeyPoint> &keypoints_1 ,std::vector<cv::KeyPoint> &keypoints_2,std::vector<cv::DMatch> &good_matches)
{
  //SURF feature
  cv::Ptr<cv::xfeatures2d::SIFT> surf = cv::xfeatures2d::SIFT::create();
  cv::Mat descriptor_1 ,descriptor_2;

/*
  surf->detect(image1, keypoints_1);
  surf->detect(image2, keypoints_2);

  //提取特征点并计算特征描述子
  surf->detectAndCompute(image1, cv::noArray(), keypoints_1, descriptor_1);
  surf->detectAndCompute(image2, cv::noArray(), keypoints_2, descriptor_2);
*/

  cv::Ptr<cv::ORB> orb = cv::ORB::create();

  //檢測 orb角點位置
  orb->detect(image1, keypoints_1);
  orb->detect(image2, keypoints_2);

  //檢測 orb brief描述子
  orb->compute(image1, keypoints_1, descriptor_1);
  orb->compute(image2, keypoints_2, descriptor_2);

//-----
  std::vector<cv::DMatch> matches;
  // sift surf -> cv::NORM_L1
  // orb ->cv::NORM_HAMMING

  cv::BFMatcher matcher(cv::NORM_HAMMING);
  matcher.match(descriptor_1,descriptor_2,matches);

  double min_dist=1000, max_dist = 0;

  for(int i = 0;i <descriptor_1.rows; i++)
  {
    double dist = matches[i].distance;
    if(dist < min_dist) min_dist = dist;
    if(dist > max_dist) max_dist = dist;
  }

  for(int i =0; i<descriptor_1.rows; i++)
  {
    if(matches[i].distance <= std::max(2*min_dist,30.0))
    {
      good_matches.push_back(matches[i]);
    }
  }

}

// get image
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr img_msg , cv_bridge::CvImageConstPtr &ptr)
{

  if (img_msg->encoding == "BGR8")
  {
      sensor_msgs::Image img;
      img.header = img_msg->header;
      img.height = img_msg->height;
      img.width = img_msg->width;
      img.is_bigendian = img_msg->is_bigendian;
      img.step = img_msg->step;
      img.data = img_msg->data;
      img.encoding = "BGR8";
      ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }
  else
      ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

  cv::Mat img = ptr->image.clone();
  return img;
}


void process()
{
/*
  cv_bridge::CvImageConstPtr ptr1,ptr2;
  cv::Mat image1 , image2;

  image1 = getImageFromMsg(measurements.back().first, ptr1);
  image2 = getImageFromMsg(measurements.back().second, ptr2);

*/
  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
  std::vector<cv::DMatch> matches;

  // match feature
  find_feature_matches(imageL,imageR,keypoints_1,keypoints_2,matches);
  std::cout<<"How much pairs : "<<matches.size()<<std::endl;

  // estimate the transformation between the two photos
  cv::Mat R,t;
  pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

  std::cout<<"pose_estimation_2d2d R : "<<R<<std::endl;
  std::cout<<"pose_estimation_2d2d t : "<<t<<std::endl;

  //traingulation
  std::vector<cv::Point3d> points;
  triangulation(keypoints_1, keypoints_2, matches, R, t, points);

  //verify the re-projected error in 3d-2d
  for (int i = 0; i < matches.size(); i++)
  {
      // first photo
      cv::Point2d pt1_cam = pixel2cam(keypoints_1[matches[i].queryIdx].pt, k_b);
      cv::Point2d pt1_cam_3d(points[i].x/points[i].z, points[i].y/points[i].z);

      cv::circle(imageL,keypoints_1[i].pt,3,cv::Scalar(12,120,120),-1);
      //std::cout<<"point in the first camera frame : "<<pt1_cam<<std::endl;
      //std::cout<<"point projected from 3D "<<pt1_cam_3d<<", d="<<points[i].z<<std::endl;

      // second photo
      cv::Point2f pt2_cam = pixel2cam( keypoints_2[ matches[i].trainIdx ].pt, k_g );
      cv::Mat pt2_trans = R * (cv::Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
      pt2_trans /= pt2_trans.at<double>(2,0);
      cv::circle(imageR,keypoints_2[i].pt,3,cv::Scalar(12,120,120),-1);
      //std::cout<<"point in the second camera frame: "<<pt2_cam<<std::endl;
      //std::cout<<"point reprojected from second frame: "<<pt2_trans.t()<<std::endl;
  }

// solve the pnp to enforce the R and t
/*
  std::vector<cv::Point3f> pts_3d;
  std::vector<cv::Point2f> pts_2d;

  for(cv::DMatch m : matches)
  {
    ushort d = imageL.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
    if(d ==0)
      continue;
    float dd = d/1000.0;
    cv::Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt,k_b);
    pts_3d.push_back(cv::Point3f(p1.x*dd , p1.y*dd,dd));
    pts_2d.push_back(keypoints_2[m.trainIdx].pt);
  }

  std::cout<<"3d-2d pairs: "<<pts_3d.size()<<std::endl;

  cv::Mat r_ ,t_;
  //use the opencv of PNP, you can choose the EPNP or DLS methods
  cv::solvePnP(pts_3d,pts_2d,k_b,cv::Mat(),r_,t_,false,cv::SOLVEPNP_EPNP);

  cv::Mat R_;
  cv::Rodrigues(r_,R_);

  std::cout<<"R= "<<R_<<std::endl;
  std::cout<<"t= "<<t_<<std::endl;
*/
  cv::Mat img_RR_matches;
  // drawe the matched photo
  cv::drawMatches(imageL,keypoints_1,imageR,keypoints_2,matches,img_RR_matches, cv::Scalar(0, 255, 0));
  cv::imshow("match",img_RR_matches);

  cv::imshow("R" , imageR);
  cv::imshow("L" , imageL);
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
      frame_count++;
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
  ros::NodeHandle nh("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  imageL = cv::imread("/home/yanlong658/ITRI_ws/image/1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
  imageR = cv::imread("/home/yanlong658/ITRI_ws/image/2.jpg",CV_LOAD_IMAGE_GRAYSCALE);

  // put your image
  ros::Subscriber image1_sub = nh.subscribe<sensor_msgs::Image>("/camera1", 100, camera1);
  ros::Subscriber image2_sub = nh.subscribe<sensor_msgs::Image>("/camera2", 100, camera2);
  //cameraData1er.push_back(cameraData1);
  //cameraData2er.push_back(cameraData2);

  std::thread measurement{command};
  ros::spin();
  return 0;

}

