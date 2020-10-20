#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>

using namespace std;
using namespace cv;
using namespace ros;


cv::Mat k_t = (cv::Mat_<float>(3,3) << 866.356440, 0.0, 326.529608, 0.0, 862.655564, 263.425809, 0.0, 0.0, 1.0);
cv::Mat dis_coff = (cv::Mat_<float>(1,5) << -0.347793, 0.107340, 0.000000, 0.000722, -0.001039);

void test()
{
    Mat r_, t, R;
    cv::Mat keypoints13D = (cv::Mat_<float>(5, 3) << 12.00604, -2.8654366, 18.472504,
                                                        7.6863389, 4.9355154, 11.146358,
                                                        14.260933, 2.8320458, 12.582781,
                                                        3.4562225, 8.2668982, 11.300434,
                                                        15.316854, 3.7486348, 12.491116);
    cv::Mat keypoints22D = (cv::Mat_<float>(5, 2) << 918.1734, 196.77412,
                                                       1341.7848, 946.64838,
                                                       1309.8823, 926.85284,
                                                       1153.3813, 782.78381,
                                                       1399.0817, 488.19058);

    vector<Point3f> objectPoints = { Point3f(0,0,0),Point3f(6.5,0,0),Point3f(0,0,6.5),Point3f(0,6.5,0),Point3f(4,6.5,0),Point3f(4,6.5,7),Point3f(4,0,7),Point3f(4,8,1) };
    vector<Point2f> imagePoints = { Point2f(433,50),Point2f(512,109),Point2f(425,109),Point2f(362,106),Point2f(222,333),Point2f(480,320),Point2f(520,150),Point2f(400,170) };
    solvePnPRansac(objectPoints, imagePoints, k_t, dis_coff, r_, t, false);
    Rodrigues(r_,R);
    std::cout<<"R : "<<R<<std::endl;
    std::cout<<"t : "<<t<<std::endl;
}

int main(int argc, char** argv)
{
    init(argc, argv, "try");
    NodeHandle nh;
    console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    test();
    spin();
    return 0;
}
