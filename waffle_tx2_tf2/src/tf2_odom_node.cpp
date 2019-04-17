#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

class OdomCaster
{
  public:
    OdomCaster()
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, 0);
      current_ = tf2::Transform(q);
      sub_ = node_.subscribe("odom_corrections", 16,
          &OdomCaster::correctionCallback, this);
      castCurrent(ros::Time::now());
    }

    void castCurrent(const ros::Time& stamp)
    {
      geometry_msgs::TransformStamped stamped;
      stamped.header.stamp = stamp;
      stamped.header.frame_id = "map";
      stamped.child_frame_id = "odom";
      stamped.transform = tf2::toMsg(current_);
      caster_.sendTransform(stamped);
    }

    void correctionCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
    {
      tf2::Transform offset;
      tf2::fromMsg(msg->transform, offset);
      current_ *= offset;
      castCurrent(msg->header.stamp);
    }
  
  private:
    tf2_ros::TransformBroadcaster caster_;
    tf2::Transform current_;
    ros::NodeHandle node_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf2_odom_node");
  OdomCaster caster;
  ros::spin();
  return 0;
}
