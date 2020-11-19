#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

std::ofstream in;
std::mutex m_buf;
ros::Publisher pub_path,vis_pub;
nav_msgs::Path map_path;
Eigen::Vector3d t;
Eigen::Quaterniond q;
std::vector<double> x,y,z,q_w,q_x,q_y,q_z;

int count = 0;
double tmp_x , tmp_y, tmp_z ,tmp_q_w , tmp_q_x ,tmp_q_y ,tmp_q_z;


// 點到理想直線的距離
double compute_error(Eigen::Vector3d tmp_t)
{
  double t,error;
  t = (tmp_t(0) + tmp_t(1) + tmp_t(2))/3.0;
  error = std::sqrt( (-t-tmp_t(0)) * (-t-tmp_t(0)) + (-t-tmp_t(1)) * (-t-tmp_t(1)) + (-t-tmp_t(2)) * (-t-tmp_t(2)) );
  return error;
}

// 座標轉換系統
void initial_system(double position_x , double position_y , double position_z ,double orientation_w, double orientation_x, double orientation_y,
                    double orientation_z, Eigen::Vector3d &t, Eigen::Quaterniond &q)
{

        x.push_back(position_x);
        y.push_back(position_y);
        z.push_back(position_z);
        q_w.push_back(orientation_w);
        q_x.push_back(orientation_x);
        q_y.push_back(orientation_y);
        q_z.push_back(orientation_z);

        //std::cout<<"x.front()"<<x.front()<<std::endl;
        if(x.size() >29)
        {
          std::cout<<"here?"<<std::endl;
          for(int i = 0; i<30; i++)
          {
            tmp_x += x.front();
            tmp_y += y.front();
            tmp_z += z.front();
            tmp_q_w += q_w.front();
            tmp_q_x += q_x.front();
            tmp_q_y += q_y.front();
            tmp_q_z += q_z.front();

            x.pop_back();
            y.pop_back();
            z.pop_back();
            q_w.pop_back();
            q_x.pop_back();
            q_y.pop_back();
            q_z.pop_back();
          }

          tmp_x = tmp_x / 30.0;
          tmp_y = tmp_y / 30.0;
          tmp_z = tmp_z / 30.0;
          tmp_q_w = tmp_q_w / 30.0;
          tmp_q_x = tmp_q_x / 30.0;
          tmp_q_y = tmp_q_y / 30.0;
          tmp_q_z = tmp_q_z / 30.0;
          t(0) = tmp_x;
          t(1) = tmp_y;
          t(2) = tmp_z;
          q.w() = tmp_q_w;
          q.x() = tmp_q_x;
          q.y() = tmp_q_y;
          q.z() = tmp_q_z;
        }
}


void receive_data(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  visualization_msgs::Marker marker;
  geometry_msgs::PoseStamped pos_path_msg;
  Eigen::Vector3d tmp_t;
  double error;

  in.open("/home/ee405423/Desktop/data.csv",std::ios::out | std::ios::app);
  m_buf.lock();

  if(count < 31)
  {
    initial_system(-1*msg->pose.position.x,msg->pose.position.z,msg->pose.position.y,
                   msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z, t, q);
  }
  else
  {
    // 依照當前情形更改x y z 方向
    tmp_t(0) =  -1*msg->pose.position.x;
    tmp_t(1) =  msg->pose.position.z;
    tmp_t(2) =  msg->pose.position.y;
    tmp_t = q.inverse() * (tmp_t - t);
    error = compute_error(tmp_t);

    in << tmp_t(0) << "," << tmp_t(1) << "," << tmp_t(2)<< "," <<std::endl;
       //<< msg->pose.orientation.w <<"," << msg->pose.orientation.x << "," << msg->pose.orientation.y << "," << msg->pose.orientation.z<<std::endl;

    //rviz show
    pos_path_msg.header.frame_id = "world";
    pos_path_msg.header.stamp = ros::Time::now();
    pos_path_msg.pose.position.x = tmp_t(0);
    pos_path_msg.pose.position.y = tmp_t(1);
    pos_path_msg.pose.position.z = tmp_t(2);
    pos_path_msg.pose.orientation.x = msg->pose.orientation.x;
    pos_path_msg.pose.orientation.y = msg->pose.orientation.y;
    pos_path_msg.pose.orientation.z = msg->pose.orientation.z;
    pos_path_msg.pose.orientation.w = msg->pose.orientation.w;

    map_path.header.stamp = ros::Time::now();
    map_path.header.frame_id = "world";
    map_path.poses.push_back(pos_path_msg);
    pub_path.publish(map_path);

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points_and_lines";
    marker.id = 0;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 0.8; // Don't forget to set the alpha!
    marker.color.b = 0.5;
    marker.color.g = 0.5;
    marker.scale.x = 0.05;
    for(double i =0; i<2;i++ )
    {
      geometry_msgs::Point p;
      p.x = -i;
      p.y = -i;
      p.z = -i;

      marker.points.push_back(p);
    }
    vis_pub.publish(marker);
}
  in.close();
  m_buf.unlock();
  count++;
  //std::cout<<"count"<<count<<std::endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_truth");
  ros::NodeHandle nh;

  ros::Subscriber sub_data = nh.subscribe<geometry_msgs::PoseStamped>
                             ("/vrpn_client_node/RigidBody1/pose", 100, receive_data);

  pub_path = nh.advertise<nav_msgs::Path> ("/path", 100);
  vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::spin();
  return 0;

}
