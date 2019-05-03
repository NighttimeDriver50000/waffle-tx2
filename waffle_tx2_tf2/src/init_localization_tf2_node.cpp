#include <iostream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

bool run = true;

void stop(const nav_msgs::Odometry& msg)
{
  run = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "init_localization_tf2_node");
  ros::NodeHandle node;
  if (argc != 11) {
    std::cerr << "Usage: " << argv[0] << "from to x y z roll pitch yaw hz stop_topic\n";
    return 2;
  }
  std::string from = argv[1];
  std::string to = argv[2];
  double x = std::stod(argv[3]);
  double y = std::stod(argv[4]);
  double z = std::stod(argv[5]);
  double roll = std::stod(argv[6]);
  double pitch = std::stod(argv[7]);
  double yaw = std::stod(argv[8]);
  double hz = std::stod(argv[9]);
  std::string stop_topic = argv[10];
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = from;
  transform.child_frame_id = to;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;
  tf2::Quaternion q;
  q.setRPY(roll * M_PI, pitch * M_PI, yaw * M_PI);
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  ros::Subscriber stop_sub = node.subscribe(stop_topic, 1, stop); 
  tf2_ros::TransformBroadcaster caster;
  ros::Rate rate(hz);
  while (run && ros::ok()) {
    caster.sendTransform(transform);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
