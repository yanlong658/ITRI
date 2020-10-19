#include "robot_vision/parameter.h"

std::string path1;
std::string path2;

std::string path_topic1;
std::string path_topic2;


cv::Mat r_12;
cv::Mat t_12;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

    fsSettings["r_12"] >> r_12;
    fsSettings["t_12"] >> t_12;

    fsSettings["path1"] >> path1;
    fsSettings["path2"] >> path2;

    fsSettings["path_topic1"] >> path_topic1;
    fsSettings["path_topic2"] >> path_topic2;

    fsSettings.release();
}
