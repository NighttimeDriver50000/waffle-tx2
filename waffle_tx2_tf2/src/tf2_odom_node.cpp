#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

tf2_ros::TransformBroadcaster caster;
tf2::Transform current;

void correctionCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  tf2::Transform offset;
  tf2::fromMsg(msg.transform, offset);
  current *= offset;
  // Stamp and send the transform.
  geometry_msgs::TransformStamped stamped;
  stamped.header.stamp = msg.header.stamp;
  stamped.header.frame_id = "map";
  stamped.child_frame_id = "odom";
  stamped.transform = tf2::toMsg(current);
  caster.sendTransform(stamped);
}

int main(int argc, char** argv)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  current = tf2::Transform();
  ros::init(argc, argv, "tf2_odom_node");
  ros::NodeHandle node("waffle_tx2");
  ros::Subscriber sub = node.subscribe("odom_corrections", 16,
      &correctionCallback);
  ros::spin();
  return 0;
}
