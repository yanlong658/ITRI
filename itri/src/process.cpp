#include "itri/process.h"

imageProcess::imageProcess():cameraData1(195.0,305.0), cameraData2(376.0,308.0),count(0)
{

}

// input[x , y, 1]
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

//pixel frame translate to camera frame
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
  //SURF feature
  cv::Mat descriptor_1 ,descriptor_2;

/*
  cv::Ptr<cv::xfeatures2d::SIFT> surf = cv::xfeatures2d::SIFT::create();
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
    if(matches[i].distance <= std::max(2*min_dist,30.0))
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

  // 這邊點可以distortion
  for(int i =0; i<(int) matches.size();i++)
  {
    points1.push_back(keypoint_1[matches[i].queryIdx].pt);
    points2.push_back(keypoint_2[matches[i].trainIdx].pt);
  }

  // 計算本質矩陣
  cv::Mat essential_matrix;
  // CV_FM_RANSAC for the RANSAC algorithm. N≥8
  // CV_FM_LMEDS for the LMedS algorithm. N≥8
  essential_matrix = cv::findEssentialMat(points1,points2, k_b,CV_FM_LMEDS);
  //std::cout<<"essential matrix : "<<essential_matrix<<std::endl;

  // 這邊的k假設兩個frame有一樣的內參
  cv::recoverPose(essential_matrix, points1,points2,k_b,R, t);
}

// tranglate the point
void imageProcess::triangulation(const std::vector<cv::KeyPoint> &keypoint_1, const std::vector<cv::KeyPoint> &keypoint_2,
                   const std::vector<cv::DMatch> &matches, const cv::Mat R , const cv::Mat t, std::vector<cv::Point3f> &points)
{
  cv::Mat T1 = (cv::Mat_<float>(3,4) <<
          1.0,0.0,0.0,0.0,
          0.0,1.0,0.0,0.0,
          0.0,0.0,1.0,0.0);

  cv::Mat T2;
  if(count == 0){
      std::cout<<"count : "<<count<<std::endl;
      T2 = (cv::Mat_<float>(3,4) <<
      R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0)/5.0,
      R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0)/5.0,
      R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0)/5.0);
  }else {
      std::cout<<"count : "<<count<<std::endl;
      T2 = (cv::Mat_<float>(3,4) <<
      R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0)/5.0,
      R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0)/5.0,
      R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0)/5.0);
  }

  std::vector<cv::Point2f> pts_1,pts_2;

  // 這邊可以distortion
  for(cv::DMatch m:matches)
  {
    // 將像素座標轉換到相機座標
    pts_1.push_back(distortion_9(pixel2cam_9(keypoint_1[m.queryIdx].pt,k_b)));
    pts_2.push_back(distortion_10(pixel2cam_10(keypoint_2[m.queryIdx].pt,k_g)));
  }

  for(int i =0;i<pts_1.size();i++)
  {
      std::cout<<"pts_1[i] : "<<pts_1[i]<<std::endl;
  }
  cameraData1er.clear();cameraData2er.clear();
  // 測試data是cameraData1 and cameraData2
  cameraData1er.push_back(distortion_9(pixel2cam_9(cameraData1,k_b)));
  cameraData2er.push_back(distortion_10(pixel2cam_10(cameraData2,k_g)));

  // std::cout<<"pts_1.size() : "<<pts_1.size()<<std::endl;
  //std::cout<<"cameraData1er.size() : "<<cameraData1er.size()<<std::endl;

  cv::Mat pts_4d, test_data;
  // 三角化完的點在word frame上
  cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
  //std::cout<<"cameraData1er : "<<cameraData1er.size()<<std::endl;
  cv::triangulatePoints(T1, T2, cameraData1er, cameraData2er, test_data);


  for(int i =0;i<pts_4d.cols;i++)
  {
    cv::Mat x = pts_4d.col(i);
    //std::cout<<"x : "<<x<<std::endl;
    x /= x.at<float>(3,0);
    cv::Point3d P(x.at<float>(0,0), x.at<float>(1,0),x.at<float>(2,0));
    std::cout<<"P : "<<P<<std::endl;
    points.push_back(P);
  }
 //test data
/*
  std::cout<<"test_data.cols : "<<test_data.cols<<std::endl;
  for(int i =0;i<test_data.cols;i++)
  {
    cv::Mat x = test_data.col(i);
    //std::cout<<"x : "<<x<<std::endl;
    x /= x.at<float>(3,0);
    cv::Point3d P(x.at<float>(0,0), x.at<float>(1,0),x.at<float>(2,0));
    std::cout<<"P test data: "<<P<<std::endl;
    points.push_back(P);
  }
*/
  count++;
}

// 3f應該是要world frame之下 point2f是要第二張camera的點
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
