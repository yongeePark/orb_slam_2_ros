#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <iostream>


class VelocityMeasure
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  double maxvelocity_;
  int velocity_count_;
  double velocity_sum_;
public:
  VelocityMeasure() : maxvelocity_(0), velocity_count_(0),velocity_sum_(0)
  {
    sub_ = nh_.subscribe("/scout/mavros/local_position/odom",1,&VelocityMeasure::Callback,this);
    std::cout.precision(3);
  }
  //callback
  void Callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    velocity_count_++;
    double x = msg->twist.twist.linear.x;
    double y = msg->twist.twist.linear.y;
    double z = msg->twist.twist.linear.z;

    double current_velocity = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
    velocity_sum_ += current_velocity;

    if(current_velocity > maxvelocity_)
    {
        maxvelocity_ = current_velocity;
    }
    if((velocity_count_% 10)==0)
    {
      std::cout<<"max velocity : "<<maxvelocity_<<" m/s     ";
      std::cout<<"average velocity : "<<velocity_sum_ / velocity_count_<<" m/s"<<std::endl;
    }

  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pto_node");

  VelocityMeasure velocitymeasure;
  std::cout<<"initiating velocitymeasure node!"<<std::endl;
  ros::spin();
  return 0;


}

