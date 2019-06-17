#include <iostream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tx2_navigation_node");
  ros::NodeHandle node;
  if (argc != 0) {
    std::cerr << "Usage: " << "You did something wrong.\n";
    return 2;
  }
  ros::spin();
  return 0;
}
