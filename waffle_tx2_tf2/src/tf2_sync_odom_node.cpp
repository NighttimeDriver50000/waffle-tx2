#include <iostream>
#include <optional>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

geometry_msgs::PoseStamped extract_pose(const nav_msgs::Odometry& odom)
{
  geometry_msgs::PoseStamped pose;
  pose.header = odom.header;
  pose.pose = odom.pose.pose;
  return pose;
}

geometry_msgs::TransformStamped stamp_fake(const tf2::Transform& transform)
{
  geometry_msgs::TransformStamped stamped;
  stamped.header.stamp = ros::Time(0);
  stamped.header.frame_id = "";
  stamped.child_frame_id = "";
  tf2::convert(transform, stamped.transform);
  return stamped;
}

class OdomSync
{
  public:
    OdomSync(const std::string& from_frame, const std::string& from_odom,
        const std::string& to_frame, const std::string& to_odom,
        const tf2::Transform& transform)
      : from_frame_(from_frame), from_odom_(from_odom), to_frame_(to_frame),
        to_odom_(to_odom), transform_(transform), listener_(buffer_),
        from_odom_filter_(from_odom_sub_, buffer_, from_frame_, 1, node_),
        to_odom_filter_(to_odom_sub_, buffer_, to_frame_, 1, node_)
    {
      from_odom_sub_.subscribe(node_, from_odom_, 1);
      to_odom_sub_.subscribe(node_, to_odom_, 1);
      from_odom_filter_.registerCallback(
          boost::bind(&OdomSync::from_odom_callback, this, _1));
      to_odom_filter_.registerCallback(
          boost::bind(&OdomSync::to_odom_callback, this, _1));
      from_odom_pose_.header.stamp = ros::Time::now();
      from_odom_pose_.header.frame_id = "";
      from_odom_pose_.pose.position.x = 0;
      from_odom_pose_.pose.position.y = 0;
      from_odom_pose_.pose.position.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, 0);
      from_odom_pose_.pose.orientation.x = q.x();
      from_odom_pose_.pose.orientation.y = q.y();
      from_odom_pose_.pose.orientation.z = q.z();
      from_odom_pose_.pose.orientation.w = q.w();
      to_odom_pose_ = from_odom_pose_;
    }

    void broadcast()
    {
      geometry_msgs::TransformStamped stamped;
      stamped.header.stamp = ros::Time::now();
      stamped.header.frame_id = from_frame_;
      stamped.child_frame_id = to_frame_;
      stamped.transform.translation.x = to_odom_pose_.pose.position.x
        - from_odom_pose_.pose.position.x;
      stamped.transform.translation.y = to_odom_pose_.pose.position.y
        - from_odom_pose_.pose.position.y;
      stamped.transform.translation.z = to_odom_pose_.pose.position.z
        - from_odom_pose_.pose.position.z;
      tf2::Quaternion from_q;
      tf2::convert(from_odom_pose_.pose.orientation, from_q);
      tf2::Quaternion to_q;
      tf2::convert(to_odom_pose_.pose.orientation, to_q);
      tf2::convert(to_q * from_q.inverse(), stamped.transform.rotation);
#if 0
      std::cerr << "from(" << from_frame_ << ", " << from_odom_ << ") to("
        << to_frame_ << ", " << to_odom_ << ") Sending: " << stamped << "\n";
#endif
      broadcaster_.sendTransform(stamped);
    }

    void from_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      geometry_msgs::PoseStamped base = buffer_.transform(
          extract_pose(*msg), from_frame_);
      geometry_msgs::PoseStamped pose;
      tf2::doTransform(base, pose, stamp_fake(transform_));
      from_odom_pose_ = pose;
      broadcast();
    }

    void to_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      to_odom_pose_ = buffer_.transform(extract_pose(*msg), to_frame_);
      broadcast();
    }

  private:
    std::string from_frame_;
    std::string from_odom_;
    std::string to_frame_;
    std::string to_odom_;
    tf2::Transform transform_;
    ros::NodeHandle node_;
    tf2_ros::TransformBroadcaster broadcaster_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    message_filters::Subscriber<nav_msgs::Odometry> from_odom_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> to_odom_sub_;
    tf2_ros::MessageFilter<nav_msgs::Odometry> from_odom_filter_;
    tf2_ros::MessageFilter<nav_msgs::Odometry> to_odom_filter_;
    geometry_msgs::PoseStamped from_odom_pose_;
    geometry_msgs::PoseStamped to_odom_pose_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf2_sync_odom_node");
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
  tf2::Quaternion q;
  q.setRPY(roll * M_PI, pitch * M_PI, yaw * M_PI);
  tf2::Transform transform(q, tf2::Vector3(x, y, z));
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
  OdomSync odom_sync(from_frame, from_odom, to_frame, to_odom, transform);
  ros::spin();
  return 0;
}
