#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOccupancyOcTree.h>	


using namespace std;
using namespace octomap;

ros::Publisher *markers_pub_ptr;

//Parameters:
double  m_colorFactor;

std_msgs::ColorRGBA heightMapColor(double h){

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

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg){
  
  visualization_msgs::MarkerArray occupiedNodesVis;
  geometry_msgs::Point cubeCenter;
  std_msgs::ColorRGBA m_color;
  
  m_color.r = 0.0;
  m_color.g = 0.0;
  m_color.b = 1.0;
  m_color.a = 1.0;
  
  AbstractOcTree* tree = msgToMap(*msg);

  if(tree){
    
    OcTree* octree = dynamic_cast<OcTree*>(tree);
    
    for(OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it){
      if(octree->isNodeOccupied(*it)){
	unsigned idx = it.getDepth();
	cubeCenter.x = it.getX();
	cubeCenter.y = it.getY();
	cubeCenter.z = it.getZ();
	occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
	double minX, minY, minZ, maxX, maxY, maxZ;
	octree->getMetricMin(minX, minY, minZ);
	octree->getMetricMax(maxX, maxY, maxZ);
	double h = (1.0 - std::min(std::max((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * m_colorFactor;
	occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
      }
    }
    
    for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i){
      double size = octree->getNodeSize(i);
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

  }else{
    ROS_ERROR("Octomap conversion error!");
    exit(1);
  }
  
  markers_pub_ptr->publish(occupiedNodesVis);
}

int main(int argc,char *argv[])
{
  
  ros::init(argc, argv, "froctomap_markers");
  ros::NodeHandle n;

  n.param("colorFactor", m_colorFactor, 0.8);

  //Subscriber:
  ros::Subscriber sub_octo = n.subscribe("/octomap_binary", 1000, octomapCallback);
  
  //Publisher:
  ros::Publisher markers_pub = n.advertise<visualization_msgs::MarkerArray>("/octomap_binary_markers", 100);
  markers_pub_ptr = &markers_pub;

  ros::spin();
  
  return 0;
  
}



