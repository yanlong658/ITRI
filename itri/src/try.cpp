#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <eigen3/Eigen/Dense>
# define M_PI 3.14159265358979323846

using namespace std;
using namespace cv;
using namespace ros;

void degee2rad(Eigen::Vector3d &ea)
{
    ea[0] = ea[0] *  M_PI /180.0;
    ea[1] = ea[1] *  M_PI /180.0;
    ea[2] = ea[2] *  M_PI /180.0;
}

void rad2degree(Eigen::Vector3d &eulerAngle1)
{
    eulerAngle1[0] = eulerAngle1[0]*180.0/M_PI;
    eulerAngle1[1] = eulerAngle1[1]*180.0/M_PI;
    eulerAngle1[2] = eulerAngle1[2]*180.0/M_PI;
}

void euler2rotation()
{
    // roll = 2.24, pitch = 5.52, yaw = 76.73
    Eigen::Vector3d ea(2.24 , 5.52 ,  76.73 );
    degee2rad(ea);

    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    cout << "rotation matrix3 =\n" << rotation_matrix3 << endl;

}

void rotation2euler()
{
    Eigen::Matrix3d rotattion;
    rotattion << -0.9209, -0.1272, -0.3683,
                 -0.0762, 0.98576, -0.1498,
                  0.3821, -0.1098, -0.917;

    Eigen::Vector3d eulerAngle1 = rotattion.eulerAngles(2,1,0);
    rad2degree(eulerAngle1);
        cout << "eulerAngle1, z y x: " << eulerAngle1 << endl;

}

int main(int argc, char** argv)
{
    init(argc, argv, "try");
    NodeHandle nh;
    console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    euler2rotation();
    rotation2euler();
    spin();
    return 0;
}
