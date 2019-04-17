#include <iostream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf2_node");
  ros::NodeHandle node;
  if (argc != 9) {
    std::cerr << "Usage: " << argv[0] << "from to x y z roll pitch yaw\n";
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
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = from;
  transform.child_frame_id = to;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  
  tf2_ros::StaticTransformBroadcaster caster;
  caster.sendTransform(transform);

  ros::spin();
  return 0;
}
