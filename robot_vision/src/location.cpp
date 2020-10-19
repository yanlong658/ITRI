#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"
#include <geometry_msgs/Pose2D.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/Image.h>

using namespace cv;
using namespace std;

//找轮廓
class CONTOUR
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_img_;
    ros::Publisher image_pub_pos_;
    std::string sub_image_path;

public:
    CONTOUR():it_(nh_), nh_("~")
    {
        nh_.getParam("sub_image_path", sub_image_path);
        image_sub_ = it_.subscribe(sub_image_path, 1, &CONTOUR::convert_callback, this);
        image_pub_img_ = it_.advertise("contour_output", 1);
        image_pub_pos_ = nh_.advertise<geometry_msgs::Pose2D>("position_center_of_mass", 1);
        cv::namedWindow("Contours Image");
    }
    ~CONTOUR()
    {
        cv::destroyWindow("Contours Image");
    }

    void convert_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

        findcontours(cv_ptr->image);
    }

    void findcontours(Mat srcimg)
    {
        vector<vector<Point> > g_vContours;
        vector<Vec4i> g_vHierarchy;

        medianBlur(srcimg, srcimg, 5);
        Canny(srcimg, srcimg, 100, 255, 3);
        findContours(srcimg, g_vContours, g_vHierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

        vector<Moments> mu(g_vContours.size());                                                                   //计算矩
        for (unsigned int i = 0; i < g_vContours.size(); i++)
        {
            mu[i] = moments(g_vContours[i], false);
        }
        vector<Point2f> mc(g_vContours.size());                                                                   //计算中心矩
        for (unsigned int i = 0; i < g_vContours.size(); i++)
        {
            mc[i] = Point2f(static_cast<float>(mu[i].m10 / mu[i].m00), static_cast<float>(mu[i].m01 / mu[i].m00));
        }

        Mat drawing = Mat::zeros(srcimg.size(), CV_8UC3);                                                         //绘制轮廓
        for (unsigned int i = 0; i < g_vContours.size(); i++)
        {
            Scalar color = Scalar(255, 0, 0);
            drawContours(drawing, g_vContours, i, color, 1, 8, g_vHierarchy, 0, Point());                         //绘制外层和内层轮廓
            circle(drawing, mc[i], 4, color, -1, 8, 0);
            printf("\t %d contour: x=%f y=%f\n", i, mc[i].x, mc[i].y);                                            //输出内容
        }

        imshow("Contours Image",drawing); //轮廓
        waitKey(5);
        img2rviz(drawing);
        geometry_msgs::Pose2D msg;
        msg.x = mc[0].x;
        msg.y = mc[0].y;
        image_pub_pos_.publish(msg);
    }

    void img2rviz(Mat img)
    {
        cv_bridge::CvImage img_out;
        img_out.encoding = sensor_msgs::image_encodings::BGR8;
        img_out.image = img;
        image_pub_img_.publish(img_out.toImageMsg());
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "contour");
    CONTOUR obj;
    ros::spin();
}

