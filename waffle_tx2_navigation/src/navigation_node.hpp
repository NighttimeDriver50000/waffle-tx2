#ifndef FILE_NAVIGATION_NODE_H
#define FILE_NAVIGATION_NODE_H

//ROS Headers
#include <ros/ros.h>
#include <ros/console.h>

//Path Headers
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

//Point Cloud Headers
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"

//Transform Headers
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

//Random Number Generation Headers
#include <random>

#define NAVIGATION_LOOPRATE 1

using namespace std;

namespace TurtleBot3Navigation
{
void cloud_callback(sensor_msgs::PointCloud2ConstPtr const& cloud_msg);
void createPathAndPub(ros::Publisher& pub);
void createCloudAndPub(ros::Publisher& pub);
sensor_msgs::PointCloud2Ptr generateRandomInViewCloud(void);
};//namespace

int main(int argc, char** argv);
#endif
