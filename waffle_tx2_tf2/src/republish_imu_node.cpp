#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

ros::Subscriber sub;
ros::Publisher pub;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  sensor_msgs::Imu imu = *msg;
  imu.header.frame_id = "imu_link";
  pub.publish(imu);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "republish_imu_node");
  ros::NodeHandle node;
  sub = node.subscribe("imu", 16, imu_callback);
  pub = node.advertise<sensor_msgs::Imu>("imu_link", 16);
  ros::spin();
  return 0;
}
