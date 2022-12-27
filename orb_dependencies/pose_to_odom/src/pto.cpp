#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <iostream>


class pto
{
private:
  ros::NodeHandle nh;
  ros::Publisher  pub_odometry_;
  ros::Publisher  pub_local_goal_;
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_global_goal_;
  double current_position_[3];
public:
  pto() : current_position_{0,0,0}
  {
    pub_odometry_ = nh.advertise<nav_msgs::Odometry>("orb_odom",1);
    pub_local_goal_ = nh.advertise<nav_msgs::Path>("/scout/local_goal",1);
    sub_pose_ = nh.subscribe("/scout/mavros/vision_pose/pose",1,&pto::callback,this);
    sub_global_goal_ = nh.subscribe("/scout/global_goal",1,&pto::global_goal_callback,this);
  }
  //callback
  void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    nav_msgs::Odometry odommsg;
    odommsg.header.stamp = ros::Time::now();
    odommsg.header.frame_id = msg->header.frame_id;
    odommsg.pose.pose.position.x = msg->pose.position.x;
    odommsg.pose.pose.position.y = msg->pose.position.y;
    odommsg.pose.pose.position.z = msg->pose.position.z;

    odommsg.pose.pose.orientation.x = msg->pose.orientation.x;
    odommsg.pose.pose.orientation.y = msg->pose.orientation.y;
    odommsg.pose.pose.orientation.z = msg->pose.orientation.z;
    odommsg.pose.pose.orientation.w = msg->pose.orientation.w;

    pub_odometry_.publish(odommsg);
    //Also, pub goal

    current_position_[0] = odommsg.pose.pose.position.x;
    current_position_[1] = odommsg.pose.pose.position.y;
    current_position_[2] = odommsg.pose.pose.position.z;

    // nav_msgs::Path pathmsg;
    // pathmsg.header.stamp = ros::Time::now();
    // pathmsg.header.frame_id="base_link";

    // geometry_msgs::PoseStamped p;
    // p.pose.position.x=10.0;
    // p.pose.position.y=0.0;
    // p.pose.position.z=0.0;
    // pathmsg.poses.push_back(p);
    // pub_goal_.publish(pathmsg);

    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"base_link","costmap"));
  }
  void global_goal_callback(const geometry_msgs::PoseStampedConstPtr &msg) {

  double global_x, global_y, global_z;
  global_x = msg->pose.position.x;
  global_y = msg->pose.position.y;
  global_z = msg->pose.position.z;

  double local_x, local_y, local_z;
  double global_yaw = 0;
  local_x = cos(-global_yaw) * (global_x - current_position_[0]) -
            sin(-global_yaw) * (global_y - current_position_[1]);
  local_y = sin(-global_yaw) * (global_x - current_position_[0]) +
            cos(-global_yaw) * (global_y - current_position_[1]);
  local_z = global_z - current_position_[2];
  
  std::cout<<"local goal : "<<local_x<<", "<<local_y<<", "<<local_z<<std::endl;
  nav_msgs::Path path;
  
  path.header.stamp=ros::Time::now();
  path.header.frame_id="base_link";
  
  geometry_msgs::PoseStamped p;
  p.pose.position.x=local_x;
  p.pose.position.y=local_y;
  p.pose.position.z=local_z;
  path.poses.push_back(p);
  pub_local_goal_.publish(path);
  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pto_node");

  pto PTO;
  std::cout<<"initiating position_to_odometry node!"<<std::endl;
  ros::spin();
  return 0;


}

