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

#define NUMBER_OF_RRT_NODES 20
#define COLLISION_DISTANCE 0.3 //meters
#define COLLISION_STEP_DISTANCE 0.05 //meters
using namespace std;

namespace TurtleBot3Navigation
{
  //Callbacks
  void cloud_callback(const sensor_msgs::PointCloud2Ptr cloud_msg);

  //Display Camera Bounding Box
  void createBoundingboxAndPub(ros::Publisher& pub, float minimumViewPoint, float maximumViewPoint, float yAxisBoundarySlope, float yAxisBoundaryIntercept, float zAxisBoundarySlope, float zAxisBoundaryIntercept);
  //Create and Publish Test Messages
  void createPathAndPub(ros::Publisher& pub);
  void createCloudAndPub(ros::Publisher& pub);

  //Create a Point Cloud 2
  sensor_msgs::PointCloud2Ptr createCloud(void);

  //Print a Point Cloud 2
  void printCloud(const sensor_msgs::PointCloud2Ptr cloud);
  
  //Calculate the Distance between two points
  float calcDistBetweenPoints(tuple<float, float, float> point1, tuple<float, float, float> point2);
  float calcDistBetweenPoints(sensor_msgs::PointCloud2Ptr cloud, size_t cloudPointIndex, tuple<float, float, float> point2);
  float calcDistBetweenPoints(float point_x1, float point_y1, float point_z1, float point_x2, float point_y2, float point_z2);

  //Find the closest point in a Point Cloud 2 from a specified point
  size_t findClosestPoint(sensor_msgs::PointCloud2Ptr cloud, size_t nodesGenerated, float point_x, float point_y, float point_z);
  size_t findClosestPoint(sensor_msgs::PointCloud2Ptr cloud, size_t nodesGenerated, tuple<float, float, float> p2);
  
  //Generate Random Floats
  float generateRandomValue(float minimum, float maximum);
  std::vector<float> generateRandomValues(float quantity, float minimum, float maximum);
  
  //Check for collision 
  bool noCollisionInSphere(sensor_msgs::PointCloud2Ptr cloud, float point_x, float point_y, float point_z, float minAllowedDistance);
  bool noCollisionInCylinder(sensor_msgs::PointCloud2Ptr cloud, tuple<float, float, float> point1, tuple<float, float, float> point2, float minAllowedDistance);
  bool noCollision(sensor_msgs::PointCloud2Ptr cloud, float point_x1, float point_y1, float point_z1, float point_x2, float point_y2, float point_z2, float minAllowedDistance, float minAllowedStepDistance);

  //Dot Product functions
  float point3dDot(float p1_x, float p1_y, float p1_z, float p2_x, float p2_y, float p2_z);
  float point3dDot(tuple<float, float, float> p1, tuple<float, float, float> p2);

  //Cross Product functions
  tuple<float, float, float> point3dCross(tuple<float, float, float> p1, tuple<float, float, float> p2);
  tuple<float, float, float> point3dCross(float p1_x, float p1_y, float p1_z, float p2_x, float p2_y, float p2_z);
  
  //Genorate RRT path in camera view
  sensor_msgs::PointCloud2Ptr generateRRTPathInView(float minimumViewPoint, float maximumVeiwPoint, float yAxisBoundarySlope, float yAxisBoundaryIntercept, float zAxisBoundarySlope, float zAxisBoundaryIntercept);

  //Bool to wait for Obsticle Cloud before starting Program
  bool hasNoObsticleCloud = true;
  //Point cloud 2, populated by callback, to represent obsticles
  sensor_msgs::PointCloud2Ptr obstacleCloud;


};//namespace

int main(int argc, char** argv);
#endif
