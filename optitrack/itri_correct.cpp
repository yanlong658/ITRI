#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <string>

std::vector<double> _matrix;
std::vector<Eigen::Vector3d> _receive_data;
std::ofstream correct_csv;

Eigen::Vector3d opti,laser;

std::string opti_after_rotation_path,corrected_path;

void receive_excel();

bool  readParameter(ros::NodeHandle &nh)
{
  bool result = true;
  // get opti and laser variable
  if (!nh.getParam("opti_x", opti(0)) || !nh.getParam("opti_y", opti(1)) || !nh.getParam("opti_z", opti(2)))
  {
      ROS_ERROR("Failed to get param opti");
      result = false;
  }

  if (!nh.getParam("laser_x", laser(0)) || !nh.getParam("laser_y", laser(1)) || !nh.getParam("laser_z", laser(2)))
  {
      ROS_ERROR("Failed to get param laser");
      result = false;
  }

  if(!nh.getParam("opti_after_rotation_path", opti_after_rotation_path) || !nh.getParam("corrected_path", corrected_path))
  {
    ROS_ERROR("Failed to get param csv path");
    result = false;
  }
  receive_excel();
}

// receive the optitrack data from excel.
void receive_excel()
{
    std::fstream file;
    // 這邊的optirack是以機械手臂為world frame
    file.open(opti_after_rotation_path);
    std::string line;
    while(std::getline( file, line,'\n'))  //讀檔讀到跳行字元
    {
      std::istringstream templine(line); // string 轉換成 stream
      std::string data;
      while (getline( templine, data,',')) //讀檔讀到逗號
      {
        _matrix.push_back(atof(data.c_str()));  //string 轉換成數字
      }
    }
    file.close();
}

Eigen::Quaterniond compute_quaternion()
{
  Eigen::Vector3d _t;
  Eigen::Quaterniond relative_q;
  Eigen::Vector3d relative_n;
  double _theta;

  opti = opti/std::sqrt((opti(0)*opti(0)) + (opti(1)*opti(1)) + (opti(2)*opti(2)));
  laser = laser/std::sqrt((laser(0)*laser(0)) + (laser(1)*laser(1)) + (laser(2)*laser(2)));
  _t = opti.cross(laser);
  //ROS_INFO_STREAM("_t"<<_t(0));
  relative_n = opti.cross(laser)/std::sqrt((_t(0) * _t(0) + _t(1) * _t(1) + _t(2)*_t(2)));
  _theta = std::atan(std::sqrt((_t(0)*_t(0)) + (_t(1)*_t(1)) + (_t(2) * _t(2)))/opti.dot(laser));

  relative_q.w() = std::cos(_theta/2);
  relative_q.x() = relative_n(0) * std::sin(_theta/2);
  relative_q.y() = relative_n(1) * std::sin(_theta/2);
  relative_q.z() = relative_n(2) * std::sin(_theta/2);

  return relative_q;
}

void align(Eigen::Quaterniond relative_q)
{
  Eigen::Vector3d f;
  correct_csv.open(corrected_path,std::ios::out | std::ios::app);

  f << _receive_data.front()[0],_receive_data.front()[1],_receive_data.front()[2];
  f = relative_q * f;

  correct_csv << f(0) << "," << f(1) << "," << f(2)<<std::endl;
  _receive_data.erase(_receive_data.begin());
  correct_csv.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_truth");
  ros::NodeHandle nh;
  bool init = readParameter(nh);

  if(init)
  {

    std::cout << "\n opti" << opti(0) << " KI " << opti(1) << " KD " << opti(2) <<
                 "\n laser" << laser(0) << " KI " << laser(1) << " KD " << laser(2) << std::endl;
  }
  else
  {
    ros::shutdown();
  }

  Eigen::Quaterniond relative_q;

  //receive_excel();

  int size = _matrix.size() / 3;
  for(int i =0; i<size;i++)
  {
    Eigen::Vector3d local_data;
    local_data<< _matrix.at(i * 3 + 0) ,_matrix.at(i * 3 + 1) ,_matrix.at(i * 3 + 2);
    _receive_data.push_back(local_data);
  }
  relative_q = compute_quaternion();

  std::cout<<"size : "<<size<<std::endl;
  for(int j=0;j < size;j++)
  {
    align(relative_q);
  }
  std::cout<<"end"<<std::endl;
  ros::spin();
  return 0;

}


