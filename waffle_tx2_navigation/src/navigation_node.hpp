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

//Vector Header
#include <vector>

//Math functions Header
#include <math.h>

#define NAVIGATION_LOOPRATE 1

#define BOUND_BOX_MIN_RANGE 0.5 //meters
#define BOUND_BOX_MAX_RANGE 20.0 //meters
#define BOUND_BOX_Y_AXIS_SLOPE 0.5
#define BOUND_BOX_Y_AXIS_INTERCEPT 0 //meters
#define BOUND_BOX_Z_AXIS_SLOPE 0.4 
#define BOUND_BOX_Z_AXIS_INTERCEPT 0 //meters

#define COLLISION_DISTANCE 0.5 //meters
using namespace std;

namespace TurtleBot3Navigation
{
void cloud_callback(const sensor_msgs::PointCloud2Ptr cloud_msg);
void createBoundingboxAndPub(ros::Publisher& pub, float minimumViewPoint, float maximumViewPoint, 
      float yAxisBoundarySlope, float yAxisBoundaryIntercept, float zAxisBoundarySlope, float zAxisBoundaryIntercept);
void createPathAndPub(ros::Publisher& pub);
void createCloudAndPub(ros::Publisher& pub);
float generateRandomValue(float minimum, float maximum);
sensor_msgs::PointCloud2Ptr generateRandomInViewCloud(float minimumViewPoint, float maximumVeiwPoint, 
      float yAxisBoundarySlope, float yAxisBoundaryIntercept, float zAxisBoundarySlope, float zAxisBoundaryIntercept);

bool hasNoObsticleCloud = true;
sensor_msgs::PointCloud2Ptr obstacleCloud;
};//namespace

int main(int argc, char** argv);
#endif
