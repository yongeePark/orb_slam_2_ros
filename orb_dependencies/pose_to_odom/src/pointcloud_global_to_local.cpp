#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/PoseStamped.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h> // this is for pcl::transformPointCloud
// temp

class pc_gtl
{
private:
  ros::NodeHandle nh;
  ros::Publisher  pub;
  ros::Publisher  pub_local_pointcloud;
  
  ros::Subscriber sub_pointcloud;
  ros::Subscriber sub_pose;

  Eigen::Matrix4f matrix_gtl;
public:
  pc_gtl()
  {
    pub_local_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/BLIO/Localmap",1);

    sub_pointcloud = nh.subscribe("/orb_slam2_mono/map_points",1,&pc_gtl::callback,this);
    sub_pose = nh.subscribe("/scout/mavros/vision_pose/pose",1,&pc_gtl::PoseCallback,this);
  }
  //callback
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    geometry_msgs::PoseStamped current_pose;
    current_pose.pose.position.x = msg->pose.position.x;
    current_pose.pose.position.y = msg->pose.position.y;
    current_pose.pose.position.z = msg->pose.position.z;
    
    current_pose.pose.orientation.x = msg->pose.orientation.x;
    current_pose.pose.orientation.y = msg->pose.orientation.y;
    current_pose.pose.orientation.z = msg->pose.orientation.z;
    current_pose.pose.orientation.w = msg->pose.orientation.w;

    //set newest pose (or transform!)
    Eigen::Vector3f t(current_pose.pose.position.x,
               current_pose.pose.position.y,
               current_pose.pose.position.z);
    Eigen::Quaternionf q(current_pose.pose.orientation.w,
                         current_pose.pose.orientation.x,
                         current_pose.pose.orientation.y,
                         current_pose.pose.orientation.z);
    Eigen::Isometry3f T(q);
    T.pretranslate(t);

                        

    matrix_gtl = T.matrix();

  }
  void callback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local_vicinity(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 vicinity_msg;

    pcl::fromROSMsg(*msg, *cloud_global);

    pcl::transformPointCloud(*cloud_global, *cloud_local, matrix_gtl.inverse());
    

      for(unsigned int i=0; i< cloud_local->points.size(); i++)
    {
      pcl::PointXYZ points_raw;
      points_raw.x= cloud_local->points[i].x;
      points_raw.y= cloud_local->points[i].y;
      points_raw.z= cloud_local->points[i].z;
      double dist = sqrt(pow((points_raw.x),2) + pow((points_raw.y),2));
      if (abs(points_raw.z)<=5){ // default : 0.1
        
        if (dist<=20){
          cloud_local_vicinity->points.push_back(points_raw);
        } 
      }
    }
    pcl::toROSMsg(*cloud_local_vicinity, vicinity_msg);
    vicinity_msg.header.frame_id = "base_link";
    vicinity_msg.header.stamp = ros::Time::now();
    pub_local_pointcloud.publish(vicinity_msg); 






  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_gtl");

  pc_gtl GTL;
  std::cout<<"initiating pc_gtl node!"<<std::endl;
  ros::spin();
  return 0;


}

