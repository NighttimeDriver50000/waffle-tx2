#include "navigation_node.hpp"

namespace TurtleBot3Navigation
{
  void cloud_callback(sensor_msgs::PointCloud2ConstPtr const& msg)
  {
    ROS_INFO("Processing Cloud Message!");
    //Update the point cloud used for planning
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"),
                                                 iter_y(*msg, "y"),
                                                 iter_z(*msg, "z");
    int i = 0;
    while(iter_x != iter_x.end())
    {
      //ROS_INFO_STREAM("Point " << i << ": (" << *(iter_x) << ", " << *(iter_y) << ", " << *(iter_z) << ")");
      ++iter_x; ++iter_y; ++iter_z; ++i;
    }  
  }
  
  void createPathAndPub(ros::Publisher& pub)
  {
    nav_msgs::Path path; 
    geometry_msgs::PoseStamped pose;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    for(int i = 0; i < 15; i++)
    {
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map";
      pose.pose.position.x = i;
      pose.pose.position.y = 0;
      pose.pose.position.z = 0;
      path.poses.push_back(pose);
    }
    pub.publish(path);
  }
  void createCloudAndPub(ros::Publisher& pub)
  {
    sensor_msgs::PointCloud2Ptr cloud;
    cloud.reset(new sensor_msgs::PointCloud2);
    cloud->header.frame_id = "map";
    cloud->is_bigendian = false;
    cloud->is_dense = false;
    cloud->width = 1;
    cloud->height = 10;
    
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, 
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::PointField::FLOAT32);
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x"),
      iter_y(*cloud, "y"),
      iter_z(*cloud, "z"),
      iter_rgb(*cloud,"rgb");
   
    for (size_t i = 0; i < (cloud->width)*(cloud->height); ++i) 
    {
      *iter_x = (float) i;
      *iter_y = (float) i;
      *iter_z = (float) i;
      *iter_rgb = 0xFF0000;
      ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
    }
  pub.publish(cloud); 
  }
  
  float generateRandomValue(float minimum, float maximum)
  {
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dis(minimum, maximum); // distribution in range [min, max]
    return dis(rng);
  } 
  sensor_msgs::PointCloud2Ptr generateRandomInViewCloud(void)
  {
    sensor_msgs::PointCloud2Ptr cloud;
    cloud.reset(new sensor_msgs::PointCloud2);
    cloud->header.frame_id = "map";
    cloud->is_bigendian = false;
    cloud->is_dense = false;
    cloud->width = 1;
    cloud->height = 10;
    
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, 
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::PointField::FLOAT32);
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x"),
      iter_y(*cloud, "y"),
      iter_z(*cloud, "z"),
      iter_rgb(*cloud,"rgb");
   
    for (size_t i = 0; i < (cloud->width)*(cloud->height); ++i) 
    {
      *iter_x = (float) i;
      *iter_y = (float) i;
      *iter_z = (float) i;
      *iter_rgb = 0xFF0000;
      ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
    }
  return cloud;
  }
} //Namespace 

using namespace TurtleBot3Navigation;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tx2_navigation_node");
  ros::NodeHandle node;
  ros::Rate loop_rate(NAVIGATION_LOOPRATE);
 
  ros::Duration(5).sleep(); // sleep in seconds

  ros::Publisher path_pub = node.advertise<nav_msgs::Path>("path",1);
  ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("cloud",1);
  ros::Subscriber pointCloud_sub = node.subscribe("/zed/zed_node/point_cloud/cloud_registered", 1, cloud_callback); 
  while(ros::ok())
  {
    ros::spinOnce();
    ROS_INFO("Spin Once!");
    createPathAndPub(path_pub);
    createCloudAndPub(cloud_pub);
    loop_rate.sleep();
  }
  return 0;
}
