#include <iostream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf2_sync_odom_node");
  ros::NodeHandle node;
  if (argc != 5 && argc != 11) {
    std::cerr << "Usage: " << argv[0]
      << "from_frame from_odom to_frame to_odom [x y z roll pitch yaw]\n";
    return 2;
  }
  std::string from_frame = argv[1];
  std::string from_odom = argv[2];
  std::string to_frame = argv[3];
  std::string to_odom = argv[4];
  double x = 0;
  double y = 0;
  double z = 0;
  double roll = 0;
  double pitch = 0;
  double yaw = 0;
  if (argc == 11) {
    x = std::stod(argv[5]);
    y = std::stod(argv[6]);
    z = std::stod(argv[7]);
    roll = std::stod(argv[8]);
    pitch = std::stod(argv[9]);
    yaw = std::stod(argv[10]);
  }
  geometry_msgs::Transform transform;
  transform.translation.x = x;
  transform.translation.y = y;
  transform.translation.z = z;
  tf2::Quaternion q;
  q.setRPY(roll * M_PI, pitch * M_PI, yaw * M_PI);
  transform.rotation.x = q.x();
  transform.rotation.y = q.y();
  transform.rotation.z = q.z();
  transform.rotation.w = q.w();
  //  Below, frames are represented as functions representing applying the
  //  inverse of the transform from the frame's parent to the frame. The base
  //  frame is `map`.
  //
  //  First, the localizer. `odom` is this node.
  //  imu_odom(base_link(/tb3/odometry/filtered))
  //      = odom(base_footprint(/tb3/odom))
  //
  //  And the ZED. `zed_map` is this node, and `zed_camera_center` is the
  //  (inverse of the---following the rules above) passed static transform:
  //  odom(base_footprint(/tb3/odom)
  //      = zed_camera_center(zed_map(zed_odom(/zed/zed_node/odom)))
}
