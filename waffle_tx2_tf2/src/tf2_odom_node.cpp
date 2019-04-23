#include <string>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

class OdomCaster
{
  public:
    OdomCaster(const std::string& odom_frame)
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, 0);
      current_ = tf2::Transform(q);
      odom_frame_ = odom_frame;
      timer_ = node_.createTimer(ros::Duration(0.02),
          &OdomCaster::castCurrent, this);
      sub_ = node_.subscribe("odom_corrections", 16,
          &OdomCaster::correctionCallback, this);
    }

    void castCurrent(const ros::TimerEvent& event)
    {
      geometry_msgs::TransformStamped stamped;
      stamped.header.stamp = ros::Time::now();
      stamped.header.frame_id = "map";
      stamped.child_frame_id = odom_frame_;
      stamped.transform = tf2::toMsg(current_);
      caster_.sendTransform(stamped);
    }

    void correctionCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
    {
      tf2::Transform offset;
      tf2::fromMsg(msg->transform, offset);
      current_ *= offset;
    }
  
  private:
    tf2_ros::TransformBroadcaster caster_;
    tf2::Transform current_;
    ros::NodeHandle node_;
    std::string odom_frame_;
    ros::Timer timer_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf2_odom_node");
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << "odom_frame\n";
    return 2;
  }
  OdomCaster caster(argv[1]);
  ros::spin();
  return 0;
}
