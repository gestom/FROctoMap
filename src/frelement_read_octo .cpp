#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOccupancyOcTree.h>	
#include "CFremenGrid.h"
#include "CTimer.h"


//The TF must be corrected and then these values must be changed!!!
#define MAX_X 54
#define MAX_Y 42
#define MAX_Z 94

#define LIM_MIN_X -2.6
#define LIM_MIN_Y -2.0
#define LIM_MIN_Z 0.0
#define LIM_MAX_X 0.05
#define LIM_MAX_Y 0.05
#define LIM_MAX_Z 4.7

using namespace std;
using namespace octomap;

CFremenGrid *grid_ptr;

std_msgs::ColorRGBA heightMapColor(double h)
{

	std_msgs::ColorRGBA color;
	color.a = 1.0;
	// blend over HSV-values (more colors)

	double s = 1.0;
	double v = 1.0;

	h -= floor(h);
	h *= 6;
	int i;
	double m, n, f;

	i = floor(h);
	f = h - i;
	if (!(i & 1)) f = 1 - f; // if i is even
	m = v * (1 - s);
	n = v * (1 - s * f);

	switch (i) {
		case 6:
		case 0:
			color.r = v;
			color.g = n;
			color.b = m;
			break;
		case 1:
			color.r = n;
			color.g = v;
			color.b = m;
			break;
		case 2:
			color.r = m;
			color.g = v;
			color.b = n;
			break;
		case 3:
			color.r = m;
			color.g = n;
			color.b = v;
			break;
		case 4:
			color.r = n;
			color.g = m;
			color.b = v;
			break;
		case 5:
			color.r = v;
			color.g = m;
			color.b = n;
			break;
		default:
			color.r = 1;
			color.g = 0.5;
			color.b = 0.5;
			break;
	}

	return color;
}


void timeCallback(const std_msgs::Int32::ConstPtr& msg)
{

  octomap_msgs::Octomap bmap_msg;
  OcTree octree (resolution); 

  int cnt = 0;

  //Create pointcloud:
  octomap::Pointcloud octoCloud;
  sensor_msgs::PointCloud fremenCloud;
  
  geometry_msgs::Point32 test_point;


  for(double i = LIM_MIN_X; i < LIM_MAX_X; i+=resolution)
  {
    for(double j = LIM_MIN_Y; j < LIM_MAX_Y; j+=resolution)
    {
      for(double w = LIM_MIN_Z; w < LIM_MAX_Z; w+=resolution)
      {
	if(grid_ptr->retrieve(cnt, msg->data))
	{
	  test_point.x = i;
	  test_point.y = j;
	  test_point.z = w;
	  fremenCloud.points.push_back(test_point);
	  octoCloud.push_back(i,j,w);
	}
	cnt++;
      }
    }
  }
  
  //Origin
  point3d origin(0.0,0.0,0.0);

  //Insert point cloud in octomap
  octree.insertPointCloud(octoCloud, origin, -1., true, false);	

  // init visualization markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  
  unsigned int m_treeDepth = octree.getTreeDepth();
  
  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth + 1);
  
  geometry_msgs::Point cubeCenter;
  
  std_msgs::ColorRGBA m_color;
  m_color.r = 0.0;
  m_color.g = 0.0;
  m_color.b = 1.0;
  m_color.a = 1.0;
  

  for(OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it)
  {
   
    if(octree.isNodeOccupied(*it))
    {
      unsigned idx = it.getDepth();
      cubeCenter.x = it.getX();
      cubeCenter.y = it.getY();
      cubeCenter.z = it.getZ();
      occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
      double minX, minY, minZ, maxX, maxY, maxZ;
      octree.getMetricMin(minX, minY, minZ);
      octree.getMetricMax(maxX, maxY, maxZ);
      double h = (1.0 - std::min(std::max((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * m_colorFactor;
      occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
    }
    
  }
  
  for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i) 
  {
    double size = octree.getNodeSize(i);
    occupiedNodesVis.markers[i].header.frame_id = "/depth_optical_frame";
    occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].scale.x = size;
    occupiedNodesVis.markers[i].scale.y = size;
    occupiedNodesVis.markers[i].scale.z = size;
    occupiedNodesVis.markers[i].color = m_color;
  }

  
  octomap_msgs::binaryMapToMsg(octree, bmap_msg);
  
  fremenCloud.header.frame_id = "/depth_optical_frame";
  fremenCloud.header.stamp = ros::Time::now();
  
  octomap_pub_ptr->publish(bmap_msg);
  cloud_pub_ptr->publish(fremenCloud);
  markers_pub_ptr->publish(occupiedNodesVis);
  
  ROS_INFO("Octomap published!");
  
}

int main(int argc,char *argv[])
{
  
  ros::init(argc, argv, "frelement_octomap");
  ros::NodeHandle n;

  n.param("modelOrder", modelOrder, 0);
  n.param("resolution", resolution, 0.05);
  n.param("colorFactor", m_colorFactor, 0.8);
  n.param<std::string>("filename", filename, "/home/joao/catkin_ws/predict2");

  //Fremen Grid:
  CFremenGrid grid(MAX_X * MAX_Y * MAX_Z);
  load(grid);
  ROS_INFO("File loaded...");
  grid_ptr = &grid;

  //Subscribers:
  ros::Subscriber sub_time = n.subscribe("/octomap_timestamp", 1000, timeCallback);

  //Publishers::
  ros::Publisher octomap_pub = n.advertise<octomap_msgs::Octomap>("/octomap_fremen", 100);
  ros::Publisher markers_pub = n.advertise<visualization_msgs::MarkerArray>("/octomap_fremen_markers", 100);
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("/fremen_cloud", 100);

  ros::spin();
  
  return 0;
  
}



