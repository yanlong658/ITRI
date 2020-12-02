#include "itri/process.h"

imageProcess::imageProcess():cameraData1(), cameraData2()
{

}

// process the distortion
cv::Point2f imageProcess::distortion_9(cv::Point2f Point)
{
  float r = std::sqrt(Point.x * Point.x + Point.y * Point.y);
  Point.x = Point.x * (1.0+ k1_9 * r*r + k2_9 * r*r*r*r) + 2.0 * p1_9 * Point.x * Point.y + p2_9 * (r*r + 2.0* Point.x * Point.x);
  Point.y = Point.y * (1.0+ k1_9 * r*r + k2_9 * r*r*r*r) + p1_9 *(r *r + 2.0 * Point.y *Point.y) + 2.0 * p2_9 * Point.x * Point.y;

  return Point;
}

cv::Point2f imageProcess::distortion_10(cv::Point2f Point)
{
  float r = std::sqrt(Point.x * Point.x + Point.y * Point.y);
  Point.x = Point.x * (1.0+ k1_10 * r*r + k2_10 * r*r*r*r) + 2.0 * p1_10 * Point.x * Point.y + p2_10 * (r*r + 2.0* Point.x * Point.x);
  Point.y = Point.y * (1.0+ k1_10 * r*r + k2_10 * r*r*r*r) + p1_10 *(r *r + 2.0 * Point.y *Point.y) + 2.0 * p2_10 * Point.x * Point.y;

  return Point;
}

//pixel frame transformate to camera frame
cv::Point2f imageProcess::pixel2cam_9(const cv::Point2f &p, const cv::Mat &K)
{
  return cv::Point2f
    (
      (p.x - K.at<float>(0, 2)) / K.at<float>(0, 0),
      (p.y - K.at<float>(1, 2)) / K.at<float>(1, 1)
    );
}

cv::Point2f imageProcess::pixel2cam_10(const cv::Point2f &p, const cv::Mat &K)
{
  return cv::Point2f
    (
      (p.x - K.at<float>(0, 2)) / K.at<float>(0, 0),
      (p.y - K.at<float>(1, 2)) / K.at<float>(1, 1)
    );
}

// get image
cv::Mat imageProcess::getImageFromMsg(const sensor_msgs::ImageConstPtr img_msg , cv_bridge::CvImageConstPtr &ptr)
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

// find match points
void imageProcess::find_feature_matches(cv::Mat &image1 ,cv::Mat &image2,std::vector<cv::KeyPoint> &keypoints_1 ,std::vector<cv::KeyPoint> &keypoints_2,std::vector<cv::DMatch> &good_matches)
{

  cv::Mat descriptor_1 ,descriptor_2;

  //SURF feature. However the opencv3 doesn't open the surf and ros couldn't install it.
/*
  cv::Ptr<cv::xfeatures2d::SIFT> surf = cv::xfeatures2d::SIFT::create();
  surf->detect(image1, keypoints_1);
  surf->detect(image2, keypoints_2);

  surf->detectAndCompute(image1, cv::noArray(), keypoints_1, descriptor_1);
  surf->detectAndCompute(image2, cv::noArray(), keypoints_2, descriptor_2);
*/

  // Orb feature
  cv::Ptr<cv::ORB> orb = cv::ORB::create(100,1.2f,8);

  // detect the feature position
  orb->detect(image1, keypoints_1);
  orb->detect(image2, keypoints_2);

  // detect the orb brief
  orb->compute(image1, keypoints_1, descriptor_1);
  orb->compute(image2, keypoints_2, descriptor_2);

  std::vector<cv::DMatch> matches;
  // sift surf -> cv::NORM_L1
  // orb ->cv::NORM_HAMMING

  // flann match function
  //cv::FlannBasedMatcher matcher;

  cv::BFMatcher matcher(cv::NORM_L2);
  matcher.match(descriptor_1, descriptor_2, matches);

  double min_dist=5000.0, max_dist = 0.0;

  for(int i = 0;i <descriptor_1.rows; i++)
  {
    double dist = matches[i].distance;
    if(dist < min_dist) min_dist = dist;
    if(dist > max_dist) max_dist = dist;
  }

  for(int i =0; i<descriptor_1.rows; i++)
  {
    if(matches[i].distance <= std::max(2*min_dist,20.0))
    {
      good_matches.push_back(matches[i]);
    }
  }

}

// estimator the R and t
void imageProcess::pose_estimation_2d2d(std::vector<cv::KeyPoint> &keypoint_1 , std::vector<cv::KeyPoint> &keypoint_2, std::vector<cv::DMatch> &matches, cv::Mat &R,cv::Mat &t)
{
  std::vector<cv::Point2f> points1;
  std::vector<cv::Point2f> points2;

  // store the feature
  for(int i =0; i<(int) matches.size();i++)
  {
    points1.push_back(keypoint_1[matches[i].queryIdx].pt);
    points2.push_back(keypoint_2[matches[i].trainIdx].pt);
  }

  // compute the essential matrix
  cv::Mat essential_matrix;

  // CV_FM_RANSAC for the RANSAC algorithm. N≥8
  // CV_FM_LMEDS for the LMedS algorithm. N≥8
  essential_matrix = cv::findEssentialMat(points1,points2, k_b,CV_FM_LMEDS);

  // recover the pose
  cv::recoverPose(essential_matrix, points1,points2,k_b,R, t);
}

// tranglate the point
void imageProcess::triangulation(const std::vector<cv::KeyPoint> &keypoint_1, const std::vector<cv::KeyPoint> &keypoint_2,
                   const std::vector<cv::DMatch> &matches, const cv::Mat R , const cv::Mat t, std::vector<cv::Point3f> &points)
{
  cv::Mat pts_4d;
  cv::Mat T1 = (cv::Mat_<float>(3,4) <<
          1.0,0.0,0.0,0.0,
          0.0,1.0,0.0,0.0,
          0.0,0.0,1.0,0.0);

  cv::Mat T2 = (cv::Mat_<float>(3,4) <<
      R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0),
      R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1),
      R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2));

  std::vector<cv::Point2f> pts_1,pts_2;

  for(cv::DMatch m:matches)
  {
    // pixel frame to camera frame
    pts_1.push_back(pixel2cam_9(keypoint_1[m.queryIdx].pt,k_b));
    pts_2.push_back(pixel2cam_10(keypoint_2[m.queryIdx].pt,k_g));
  }

  for(int i =0;i<pts_1.size();i++)
  {
      std::cout<<"pts_1[i] : "<<pts_1[i]<<std::endl;
  }
  // traningulated the points in the world frame.
  cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

  // test data is cameraData1 and cameraData2

  for(int i =0;i<pts_4d.cols;i++)
  {
    cv::Mat x = pts_4d.col(i);
    x /= x.at<float>(3,0);
    cv::Point3d P(x.at<float>(0,0), x.at<float>(1,0),x.at<float>(2,0));
    std::cout<<"P : "<<P<<std::endl;
    points.push_back(P);
  }
}

// type of 3f is in world frame, type of the 2f is the second points.
void imageProcess::Pnp(std::vector<cv::DMatch> &matches, std::vector<cv::Point3f> &pts_3d, std::vector<cv::Point2f> &pts_2d
         ,std::vector<cv::KeyPoint> &keypoints_1, std::vector<cv::KeyPoint> &keypoints_2, cv::Mat &R, cv::Mat &t,cv::Mat imageL)
{
    std::cout<<"PNP here"<<std::endl;
    for(cv::DMatch m:matches)
    {
        pts_2d.push_back(keypoints_2[m.trainIdx].pt);
        //std::cout<<"keypoints_2[m.trainIdx].pt : "<<keypoints_2[m.trainIdx].pt<<std::endl;
    }

    std::cout<<"3d-2d pairs: "<<pts_3d.size()<<std::endl;
    std::cout<<"pts_2d_ size: "<<pts_2d.size()<<std::endl;

    cv::Mat r_;
    //use the opencv of PNP, you can choose the EPNP or DLS methods
    // cv::solvePnP(物理點座標, 特徵點圖座標, 相機內參, 相機distortion, output rotation, output translation)
    cv::solvePnP(pts_3d, pts_2d, k_t, dis_coff, r_, t, false, cv::SOLVEPNP_DLS);
    cv::Rodrigues(r_,R);
}
