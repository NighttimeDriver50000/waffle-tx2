#include "navigation_node.hpp"

namespace TurtleBot3Navigation
{
  void cloud_callback(const sensor_msgs::PointCloud2Ptr msg)
  {
    hasNoObsticleCloud = false;
    assert (msg != 0);
    //ROS_INFO("Processing Cloud Message!");

    //Update the point cloud used for planning
    obstacleCloud = msg;
    //ROS_INFO("DOES ASSIGNMENT WORK?");
  }
  
  void createBoundingboxAndPub(ros::Publisher& pub, float minimumViewPoint, float maximumViewPoint, 
      float yAxisBoundarySlope, float yAxisBoundaryIntercept, float zAxisBoundarySlope, float zAxisBoundaryIntercept)
  {
    nav_msgs::Path path; 
    geometry_msgs::PoseStamped pose;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "base_link";
      
    std::vector<float> extremes(6,0);

    extremes[0] = minimumViewPoint;
    extremes[1] = maximumViewPoint;
    extremes[2] = yAxisBoundarySlope*minimumViewPoint+yAxisBoundaryIntercept;
    extremes[3] = yAxisBoundarySlope*maximumViewPoint+yAxisBoundaryIntercept;
    extremes[4] = zAxisBoundarySlope*minimumViewPoint+zAxisBoundaryIntercept;
    extremes[5] = zAxisBoundarySlope*maximumViewPoint+zAxisBoundaryIntercept;

    //Close Left Bottom
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[0];
    pose.pose.position.y = extremes[2];
    pose.pose.position.z = -extremes[4];
    path.poses.push_back(pose);
    
    //Far Left Bottom
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[1];
    pose.pose.position.y = extremes[3];
    pose.pose.position.z = -extremes[5];
    path.poses.push_back(pose);

    //Far Right Bottom
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[1];
    pose.pose.position.y = -extremes[3];
    pose.pose.position.z = -extremes[5];
    path.poses.push_back(pose);
    
    //Close Right Bottom
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[0];
    pose.pose.position.y = -extremes[2];
    pose.pose.position.z = -extremes[4];
    path.poses.push_back(pose);
  
    //Close Left Bottom
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_base";
    pose.pose.position.x = extremes[0];
    pose.pose.position.y = extremes[2];
    pose.pose.position.z = -extremes[4];
    path.poses.push_back(pose);

    //Close Left Top
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[0];
    pose.pose.position.y = extremes[2];
    pose.pose.position.z = extremes[4];
    path.poses.push_back(pose);

    //Far Left Top
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[1];
    pose.pose.position.y = extremes[3];
    pose.pose.position.z = extremes[5];
    path.poses.push_back(pose);
    
    //Far Left Bottom
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[1];
    pose.pose.position.y = extremes[3];
    pose.pose.position.z = -extremes[5];
    path.poses.push_back(pose);
    
    //Far Left Top
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[1];
    pose.pose.position.y = extremes[3];
    pose.pose.position.z = extremes[5];
    path.poses.push_back(pose);

    //Far Right Top
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[1];
    pose.pose.position.y = -extremes[3];
    pose.pose.position.z = extremes[5];
    path.poses.push_back(pose);
    
    //Far Right Bottom
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[1];
    pose.pose.position.y = -extremes[3];
    pose.pose.position.z = -extremes[5];
    path.poses.push_back(pose);
    
    //Far Right Top
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[1];
    pose.pose.position.y = -extremes[3];
    pose.pose.position.z = extremes[5];
    path.poses.push_back(pose);

    //Close Right Top
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[0];
    pose.pose.position.y = -extremes[2];
    pose.pose.position.z = extremes[4];
    path.poses.push_back(pose);
    
    //Close Right Bottom
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[0];
    pose.pose.position.y = -extremes[2];
    pose.pose.position.z = -extremes[4];
    path.poses.push_back(pose);
    
    //Close Right Top
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[0];
    pose.pose.position.y = -extremes[2];
    pose.pose.position.z = extremes[4];
    path.poses.push_back(pose);
    
    //Close Left Top
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = extremes[0];
    pose.pose.position.y = extremes[2];
    pose.pose.position.z = extremes[4];
    path.poses.push_back(pose);

    //Publish the shape
    pub.publish(path);
  }
  
  void createPathAndPub(ros::Publisher& pub)
  {
    nav_msgs::Path path; 
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "base_link";
    for(int i = 0; i < 15; i++)
    {
      geometry_msgs::PoseStamped pose; 
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "base_link";
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
    cloud->header.frame_id = "base_link";
    cloud->is_bigendian = false;
    cloud->is_dense = false;
    cloud->width = NUMBER_OF_RRT_NODES;
    cloud->height = 1;
    
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    // modifier.setPointCloud2FieldsByString(2,"xyz","rgb");
    modifier.setPointCloud2Fields(4, 
        "x", 1, sensor_msgs::PointField::FLOAT32, 
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(NUMBER_OF_RRT_NODES); 
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x"),
      iter_y(*cloud, "y"),
      iter_z(*cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> 
      iter_r(*cloud,"r"), iter_g(*cloud, "g"), iter_b(*cloud, "b");
  
    
    for (size_t i = 0; i < (cloud->width)*(cloud->height); ++i) 
    {
      *iter_x = (float) i;
      *iter_y = (float) i;
      *iter_z = (float) i;
      *iter_r = 0;
      *iter_g = 0xff;
      *iter_b = 0xff;
      //iter_rgb = 0x00ff00;
      ++iter_x; ++iter_y; ++iter_z; ++iter_r; ++iter_g; ++iter_b; 
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
  std::vector<float> generateRandomValues(float quantity, float minimum, float maximum)
  {
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dis(minimum, maximum); // distribution in range [min, max]
    std::vector<float> numbers(quantity, 0);
    for (float& i : numbers)
      i = dis(rng);
    return numbers;
  }

  float calcDistBetweenPoints(sensor_msgs::PointCloud2Ptr cloud, size_t cloudPointIndex, float point_x, float point_y, float point_z)
  {
    //Create iterators to walk through the point cloud
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x"),
      iter_y(*cloud, "y"),
      iter_z(*cloud, "z");
    iter_x = iter_x + cloudPointIndex;
    iter_y = iter_y + cloudPointIndex;
    iter_z = iter_z + cloudPointIndex;

    float x = point_x-(*iter_x);
    float xx = x*x;
    //ROS_INFO("X: %f, XX: %f", x, xx);
    float y = point_y-(*iter_y);
    float yy = y*y;
    //ROS_INFO("Y: %f, YY: %f", y, yy);
    float z = point_z-(*iter_z);
    float zz = z*z; 
    //ROS_INFO("Z: %f, ZZ: %f", z, zz);
    //use euclidean norm to find distance between the two points
    float dist = sqrt( xx + yy + zz);
    //ROS_INFO("Distance: %f", dist);
    return dist;
  }

  float calcDistBetweenPoints(float point_x1, float point_y1, float point_z1, float point_x2, float point_y2, float point_z2)
  {
    float x = point_x2-point_x1;
    float xx = x*x;
    //ROS_INFO("X: %f, XX: %f", x, xx);
    float y = point_y2-point_y1;
    float yy = y*y;
    //ROS_INFO("Y: %f, YY: %f", y, yy);
    float z = point_z2-point_z1;
    float zz = z*z; 
    //ROS_INFO("Z: %f, ZZ: %f", z, zz);
    //use euclidean norm to find distance between the two points
    float dist = sqrt( xx + yy + zz);
    //ROS_INFO("Distance: %f", dist);
    return dist;
  }
  
  float calcDistBetweenPointAndLine(float linePoint_x1, float linePoint_y1,
      float linePoint_z1, float LinePoint_x2, float linePoint_y2, float linePoint_z2,
      float point_x, float point_y, float linePoint_z)
  {
    return nan(""); 
  }
  
  size_t findClosestPoint(sensor_msgs::PointCloud2Ptr cloud, size_t nodesGenerated, float point_x, float point_y, float point_z)
  { 
    //set index to first point and minimum distance to the first point in the cloud
    size_t index = 0;
    float distMin = calcDistBetweenPoints(cloud, 0, point_x, point_y, point_z);
    //Is Safe Iter
    sensor_msgs::PointCloud2Iterator<float> iter_isSafe(*cloud, "isSafe");
    
    //walk through the point cloud checking for the minimum distance to the given point
    for(size_t i = 1; i < nodesGenerated; i++)
    {
      //use euclidean norm to find distance between the two points
      float dist = calcDistBetweenPoints(cloud, i, point_x, point_y, point_z);
      //Compare the current minimum to the newly measured distance
      if(distMin > dist && iter_isSafe[i])
      {
        //if new minimum, index and save new minimum distance
        index = i;
        distMin = dist;
      }
    }
    return index;
  }

  bool noCollisionInSphere(sensor_msgs::PointCloud2Ptr cloud, float point_x, float point_y, float point_z, float minAllowedDistance)
  {
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x"),
      iter_y(*cloud, "y"),
      iter_z(*cloud, "z");
    for (size_t i = 0; i < (cloud->width)*(cloud->height); ++i) 
    {
      float dist = calcDistBetweenPoints(cloud, i, point_x, point_y, point_z);
      if(dist < minAllowedDistance)
      {
        //ROS_INFO("Collision Detected %f < %f", dist, minAllowedDistance);
        return false;
      }
      ++iter_x; ++iter_y; ++iter_z;
    }
    return true;
  }
  bool noCollisionInCylinder(sensor_msgs::PointCloud2Ptr cloud, float point_x1, float point_y1,
      float point_z1, float point_x2, float point_y2, float point_z2, 
      float minAllowedDistance)
  {

    //Calculate the Total Distance between points
    float dist = calcDistBetweenPoints(point_x1, point_y1, point_z1, point_x2, point_y2, point_z2);

    //Calculate the Unit Vector
    float dx = (point_x2 - point_x1)/dist;
    float dy = (point_y2 - point_y1)/dist;
    float dz = (point_z2 - point_z1)/dist;

    //Create iterator to walk through Point Cloud
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x"),
      iter_y(*cloud, "y"),
      iter_z(*cloud, "z");

    //Walk Trough the Point Cloud
    for (size_t i = 0; i < (cloud->width)*(cloud->height); ++i) 
    {
      //Calculate the Vector between the start and cloud point
      float dx_sc = point_x1 - *iter_x;
      float dy_sc = point_y1 - *iter_y;
      float dz_sc = point_z1 - *iter_z;
      
      //Calculate the Vector between the goal and cloud point
      float dx_gc = point_x2 - *iter_x;
      float dy_gc = point_y2 - *iter_y;
      float dz_gc = point_z2 - *iter_z;
      
      //Calculate the Dot product between the two vectors and check if negative
      //This means the point is between the ends of the cylinder
      if( dx_sc*dx_gc + dy_sc*dy_gc + dz_sc*dz_gc < 0)
      {
        //Calculate the vector cross product with the unit vector
        float dx_cp = dy_gc*dz - dz_gc*dy;
        float dy_cp = dx_gc*dz - dz_gc*dx;
        float dz_cp = dx_gc*dy - dy_gc*dx;

        //Calculate the area of the Parallelogram formed with the unit vector (Base) and calculated vector (Side)
        //as the magnatude of the cross product of these vectors
        float area = sqrt(dx_cp*dx_cp + dy_cp*dy_cp + dz_cp*dz_cp);

        //Divide the Area of the Parallelogram to get the height
        //(unit vector distance is 1); height = area/1 = area
        
        //Check if the cloud point is inside cylinder, return false for collision.
        if(area < minAllowedDistance)
          return false;
      }
    }
    return true;
  }

  bool noCollision(sensor_msgs::PointCloud2Ptr cloud, float point_x1, float point_y1,
      float point_z1, float point_x2, float point_y2, float point_z2, 
      float minAllowedDistance, float minAllowedStepDistance)
  {
    return noCollisionInCylinder(cloud, point_x1, point_y1, point_z1, point_x2, 
        point_y2, point_z2, minAllowedDistance);
  }

  sensor_msgs::PointCloud2Ptr generateRandomInViewCloud(float minimumViewPoint, float maximumViewPoint, 
      float yAxisBoundarySlope, float yAxisBoundaryIntercept, float zAxisBoundarySlope, float zAxisBoundaryIntercept)
  {
    //ROS_INFO("Started Generating Point Cloud In Camera View!");
    //create a new point cloud with a pointer
    sensor_msgs::PointCloud2Ptr cloud;
    cloud.reset(new sensor_msgs::PointCloud2);
    //ROS_INFO("New Point Cloud Initialised!");
    //Set the frame of the point cloud
    cloud->header.frame_id = "base_link";
    //Set 
    cloud->is_bigendian = false;
    //Set
    cloud->is_dense = false;
    //Since the cloud is unordered, the width is set to 1
    cloud->width = NUMBER_OF_RRT_NODES;
    //Since the cloud is unordered, the height is set to the number of points
    cloud->height = 1;
    //Create a modifier to edit the new point cloud
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    //ROS_INFO("Created Point Cloud Modifier!");
    //Use the modifier to set the fields in the point cloud
    modifier.setPointCloud2Fields(7, "x", 1, sensor_msgs::PointField::FLOAT32, 
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::PointField::FLOAT32,
        "isSafe", 1, sensor_msgs::PointField::INT8,
        "parentIndex", 1, sensor_msgs::PointField::UINT32,
        "cost", 1, sensor_msgs::PointField::FLOAT32);
    //ROS_INFO("Created Point Cloud Fields!");
    //Create the iterators that will walk through and set each point
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x"),
      iter_y(*cloud, "y"),
      iter_z(*cloud, "z"),
      iter_rgb(*cloud,"rgb"),
      iter_isSafe(*cloud, "isSafe"),
      iter_pi(*cloud, "parentIndex"),
      iter_c(*cloud, "cost");
    //ROS_INFO("Created initial Point Cloud for generation!");
    //Set the first point as the current location of the robot
    *iter_x = 0;
    *iter_y = 0; 
    *iter_z = 0; 
    *iter_rgb = 0x0000FF;
    *iter_isSafe = true;
    *iter_pi = 0;
    *iter_c = 0;
    ROS_INFO("\n\nCreated initial Point in point cloud!");
    ROS_INFO("Point: (%f ,%f, %f) Parent Index: %f | Color: %06x", *iter_x, *iter_y, *iter_z, *iter_pi, (unsigned int) *iter_rgb);
    
    //Keep track of the amount of RRT Nodes
    size_t numberOfNodes = 1;

    //create the iterators that will acess a point given an index
    sensor_msgs::PointCloud2Iterator<float> it_x(*cloud, "x"),
      it_y(*cloud, "y"),
      it_z(*cloud, "z");
    //ROS_INFO("Created iterators for indexing!");
    //Generate the x coordinates
    vector<float> x_points = generateRandomValues(((cloud->width)*(cloud->height))-1, minimumViewPoint, maximumViewPoint);
    //ROS_INFO("Genorated x-axis coordinate points!");
    size_t pointCount = 0;
    for (float& x : x_points)
    {
      ++iter_x; ++iter_y; ++iter_z; ++iter_rgb; ++iter_pi; ++iter_c; ++numberOfNodes; ++pointCount;
      //float x = generateRandomValue(minimumViewPoint, maximumVeiwPoint);
      float yExtreme = yAxisBoundarySlope*(x)+yAxisBoundaryIntercept; 
      float y = generateRandomValue(-yExtreme, yExtreme);
      float zExtreme = zAxisBoundarySlope*(x)+zAxisBoundaryIntercept; 
      float z = generateRandomValue(-zExtreme, zExtreme);
      size_t index = findClosestPoint(cloud, numberOfNodes, x, y, z);      
      float dist = calcDistBetweenPoints(cloud, index, x, y, z);
      //ROS_INFO("Random Point: (%f ,%f, %f) Parent Index: %lu Distance: %f", x, y, z, (unsigned long) index, dist);
      
      float x_p = *(it_x + index);
      float y_p = *(it_y + index);
      float z_p = *(it_z + index);
      //ROS_INFO("Parent Point: (%f ,%f, %f)", x_p, y_p, z_p);
     
      float dx = (x - x_p)/dist;
      float dy = (y - y_p)/dist;
      float dz = (z - z_p)/dist;
      //ROS_INFO("Unit Vector: (%f ,%f, %f)", dx, dy, dz);
    
      //stringstream ss;
      //ss << "Obstacle Cloud " << obstacleCloud << endl; 
      //ROS_INFO("%s",ss.str().c_str());
      
      if(noCollision(obstacleCloud, x_p, y_p, z_p, x_p + dx, y_p + dy, z_p + dz, COLLISION_DISTANCE, COLLISION_STEP_DISTANCE))
      {
        *iter_x = x_p + dx;
        *iter_y = y_p + dy;
        *iter_z = z_p + dz;
        *iter_isSafe = true;
        *iter_pi = index;
        *iter_c = dist;
        *iter_rgb = 0x00FF00;

        ROS_INFO("Accepted Point (%lu of %lu): (%f ,%f, %f) Parent Index: %lu | color: %06x", pointCount, x_points.size(), *iter_x, *iter_y, *iter_z, (unsigned long) *iter_pi, (unsigned int) *iter_rgb);
      } else {
        *iter_x = x_p + dx;
        *iter_y = y_p + dy;
        *iter_z = z_p + dz;
        *iter_isSafe = false;
        *iter_pi = index;
        *iter_c = dist;
        *iter_rgb = 0x0000FF;

        ROS_INFO("Rejected Point (%lu of %lu): (%f ,%f, %f) Parent Index: %lu | Color: %06x", pointCount, x_points.size(), *iter_x, *iter_y, *iter_z, (unsigned long) *iter_pi, (unsigned int) *iter_rgb);
      }
    }
    return cloud;
  }


  void printCloud(const sensor_msgs::PointCloud2Ptr cloud) {
    //Create the iterators that will walk through and set each point
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x"),
      iter_y(*cloud, "y"),
      iter_z(*cloud, "z"),
      iter_rgb(*cloud,"rgb"),
      iter_isSafe(*cloud, "isSafe"),
      iter_pi(*cloud, "parentIndex"),
      iter_c(*cloud, "cost");

    unsigned long num_pts = (cloud->width)*(cloud->height);
    for (unsigned long i = 0; i < num_pts; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_pi, ++iter_rgb) 
    {
        ROS_INFO("Point (%lu of %lu): (%f ,%f, %f) Parent Index: %lu | color: %06x", i, num_pts, *iter_x, *iter_y, *iter_z, (unsigned long) *iter_pi, (unsigned int) *iter_rgb);
    }
  }  
} //Namespace 




using namespace TurtleBot3Navigation;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tx2_navigation_node");
  ros::NodeHandle node;
  ros::Rate loop_rate(NAVIGATION_LOOPRATE);

  ros::Publisher path_pub = node.advertise<nav_msgs::Path>("camera_boundry",1);
  ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("cloud_rrt",1);
  ros::Subscriber pointCloud_sub = node.subscribe("/zed/zed_node/point_cloud/cloud_registered", 1, cloud_callback); 
  //Wait for obsticle cloud
  while(hasNoObsticleCloud)
  {
    ros::spinOnce();
    ros::Duration(1).sleep(); // sleep in seconds
  }

  //Main Loop
  while(ros::ok())
  {
    ros::spinOnce();
    ROS_INFO("Spin Once!");
    //createPathAndPub(path_pub);
    createBoundingboxAndPub(path_pub, BOUND_BOX_MIN_RANGE, BOUND_BOX_MAX_RANGE, BOUND_BOX_Y_AXIS_SLOPE, BOUND_BOX_Y_AXIS_INTERCEPT,
        BOUND_BOX_Z_AXIS_SLOPE, BOUND_BOX_Z_AXIS_INTERCEPT);
   
    //ROS_INFO("Created Bounding Box!");
    //createCloudAndPub(cloud_pub);
    //sensor_msgs::PointCloud2Ptr cloud = generateRandomInViewCloud(BOUND_BOX_MIN_RANGE, BOUND_BOX_MAX_RANGE, BOUND_BOX_Y_AXIS_SLOPE, BOUND_BOX_Y_AXIS_INTERCEPT, BOUND_BOX_Z_AXIS_SLOPE, BOUND_BOX_Z_AXIS_INTERCEPT);
    createCloudAndPub(cloud_pub);
    
    // Test that cloud has all of the data here
    //printCloud(cloud);

    //Do the tings
    //ROS_INFO("Generated In View Point Cloud!");
    //cloud_pub.publish(cloud);
    //ROS_INFO("Published Point Cloud!");
    loop_rate.sleep();
  }
  return 0;
}
