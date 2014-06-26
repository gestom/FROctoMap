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

#include "fremen/UpdateGrid.h"
#include "fremen/SaveGrid.h"
#include "fremen/RetrieveOctomap.h"
#include "fremen/EstimateOctomap.h"
#include <std_msgs/String.h>

#define DIM_X 201 
#define DIM_Y 201
#define DIM_Z 101

#define LIM_MIN_X -5.0
#define LIM_MIN_Y -5.0
#define LIM_MIN_Z -1.0 
#define LIM_MAX_X 5.0
#define LIM_MAX_Y 5.0
#define LIM_MAX_Z 4.0

using namespace std;
using namespace octomap;

bool waitForPose = false;
bool waitForMap = false;
int currentMap = -1;
int numMaps = 0;
CFremenGrid *gridArray[10];
CFremenGrid *gridPtr;
ros::Publisher *octomap_pub_ptr, *retrieve_pub_ptr,*estimate_pub_ptr;

//Parameters
double resolution, m_colorFactor;
string filename;

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


void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
	if (waitForMap){
		AbstractOcTree* tree = msgToMap(*msg);
		if(tree){
			OcTree* octree = dynamic_cast<OcTree*>(tree);
			int cnt = 0;

			double minX, minY, minZ, maxX, maxY, maxZ;
			octree->getMetricMin(minX, minY, minZ);
			octree->getMetricMax(maxX, maxY, maxZ);
			printf("%f %f %f\n",minX,minY,minZ);
			printf("%f %f %f\n",maxX,maxY,maxZ);
			for(double i = LIM_MIN_X; i < LIM_MAX_X; i+=resolution){
				for(double j = LIM_MIN_Y; j < LIM_MAX_Y; j+=resolution){
					for(double w = LIM_MIN_Z; w < LIM_MAX_Z; w+=resolution){
						OcTreeNode* n = octree->search(i,j,w);
						if(n){
							if(octree->isNodeOccupied(n))
							{
								gridPtr->add(cnt,1);
							}
							else
							{
								gridPtr->add(cnt,0);
							}
						}else{
							gridPtr->add(cnt,0);
						}
						cnt++;
					}
				}
			}
			ROS_INFO("3D Grid Updated! -> Iteration: %i over %i elements in %i element grid", gridPtr->signalLength,cnt,gridPtr->numCells);
		}else{
			ROS_ERROR("Octomap conversion error!");
			exit(1);
		}
		gridPtr->signalLength++;
		delete tree;
	}
}

bool save_octomap(fremen::SaveGrid::Request  &req, fremen::SaveGrid::Response &res)
{
	gridPtr->save(req.filename.c_str(), (bool) req.lossy);
	res.size = gridPtr->signalLength;
	ROS_INFO("3D Grid saved!");
	return true;
}

bool update_octomap(fremen::UpdateGrid::Request  &req, fremen::UpdateGrid::Response &res)
{
  gridPtr->update((int) req.order, gridPtr->signalLength);
  res.size = gridPtr->signalLength;
  ROS_INFO("3D Grid updated and saved!");
  return true;
}

bool retrieve_octomap(fremen::RetrieveOctomap::Request  &req, fremen::RetrieveOctomap::Response &res)
{
  octomap_msgs::Octomap bmap_msg;
  OcTree octree (resolution);
  

  //Create pointcloud:
  octomap::Pointcloud octoCloud;
  sensor_msgs::PointCloud fremenCloud;
  
  geometry_msgs::Point32 test_point;
  int cnt = 0;
  for(double i = LIM_MIN_X; i < LIM_MAX_X; i+=resolution){
	  for(double j = LIM_MIN_Y; j < LIM_MAX_Y; j+=resolution){
		  for(double w = LIM_MIN_Z; w < LIM_MAX_Z; w+=resolution){
			  point3d ptt(i+resolution/2,j+resolution/2,w+resolution/2);
			  
			  if(gridPtr->retrieve(cnt, req.stamp))
			  {
				  octoCloud.push_back(i+resolution/2,j+resolution/2,w+resolution/2);
				  octree.updateNode(ptt,true,true);
			  }
			  cnt++;
		  }
	  }
  }
  //Update grid
  octree.updateInnerOccupancy();
  
  //init visualization markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  unsigned int m_treeDepth = octree.getTreeDepth();
  
  //each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth + 1);
  geometry_msgs::Point cubeCenter;
  
  std_msgs::ColorRGBA m_color;
  m_color.r = 0.0;
  m_color.g = 0.0;
  m_color.b = 1.0;
  m_color.a = 1.0;
  
  for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i) 
  {
    double size = octree.getNodeSize(i);
    occupiedNodesVis.markers[i].header.frame_id = "/map";
    occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].scale.x = size;
    occupiedNodesVis.markers[i].scale.y = size;
    occupiedNodesVis.markers[i].scale.z = size;
    occupiedNodesVis.markers[i].color = m_color;
  }

  for(OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it)
  {
	  if(it != NULL && octree.isNodeOccupied(*it))
	  {
		  unsigned idx = it.getDepth();
		  cubeCenter.x = it.getX();
		  cubeCenter.y = it.getY();
		  cubeCenter.z = it.getZ();
		  occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
		  double minX, minY, minZ, maxX, maxY, maxZ;
		  octree.getMetricMin(minX, minY, minZ);
		  octree.getMetricMax(maxX, maxY, maxZ);
		  double h = (1.0 - fmin(fmax((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * m_colorFactor;
		  occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
	  }
  } 
  octomap_msgs::binaryMapToMsg(octree, bmap_msg);
  
  fremenCloud.header.frame_id = "/map";
  fremenCloud.header.stamp = ros::Time::now();
  
  octomap_pub_ptr->publish(bmap_msg);
  retrieve_pub_ptr->publish(occupiedNodesVis);
  
  ROS_INFO("Octomap published!");
  
  res.result = true;

  return true;
}

bool estimate_octomap(fremen::EstimateOctomap::Request  &req, fremen::EstimateOctomap::Response &res)
{
  octomap_msgs::Octomap bmap_msg;
  OcTree octree (resolution);
  

  //Create pointcloud:
  octomap::Pointcloud octoCloud;
  sensor_msgs::PointCloud fremenCloud;
  
  geometry_msgs::Point32 test_point;
  int cnt = 0;
  char gridka[DIM_X*DIM_Y*DIM_Z];
  float estimate;
  for (int i = 0;i<DIM_X*DIM_Y*DIM_Z;i++){
	  estimate = gridPtr->estimate(i, req.stamp);
	  if(estimate >req.minProbability && estimate< req.maxProbability) gridka[i] = 1; else gridka[i] = 0;
  }
  int nn[] = {1,-1,DIM_Z,-DIM_Z,DIM_Y*DIM_Z,-DIM_Y*DIM_Z};
  for (int i = DIM_Y*DIM_Z;i<DIM_X*DIM_Y*DIM_Z-DIM_Y*DIM_Z;i++){
	  if (gridka[i] == 1){ 
		  for (int j = 0;j<6;j++){
			  if (gridka[i+nn[j]]>0) gridka[i]++;
		  }
	  }
  }
  for(double i = LIM_MIN_X; i < LIM_MAX_X; i+=resolution){
	  for(double j = LIM_MIN_Y; j < LIM_MAX_Y; j+=resolution){
		  for(double w = LIM_MIN_Z; w < LIM_MAX_Z; w+=resolution){
			  if(gridka[cnt] > req.morphology)
			  {
				  point3d ptt(i+resolution/2,j+resolution/2,w+resolution/2);
				  octoCloud.push_back(i+resolution/2,j+resolution/2,w+resolution/2);
				  octree.updateNode(ptt,true,true);
			  }
			  cnt++;
		  }
	  }
  }
  //Update grid
  octree.updateInnerOccupancy();
  
  //init visualization markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  unsigned int m_treeDepth = octree.getTreeDepth();
  
  //each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth + 1);
  geometry_msgs::Point cubeCenter;
  
  std_msgs::ColorRGBA m_color;
  m_color.r = 0.0;
  m_color.g = 0.0;
  m_color.b = 1.0;
  m_color.a = 1.0;
  
  for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i) 
  {
    double size = octree.getNodeSize(i);
    occupiedNodesVis.markers[i].header.frame_id = "/map";
    occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].scale.x = size;
    occupiedNodesVis.markers[i].scale.y = size;
    occupiedNodesVis.markers[i].scale.z = size;
    occupiedNodesVis.markers[i].color = m_color;
  }

  for(OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it)
  {
	  if(it != NULL && octree.isNodeOccupied(*it))
	  {
		  unsigned idx = it.getDepth();
		  cubeCenter.x = it.getX();
		  cubeCenter.y = it.getY();
		  cubeCenter.z = it.getZ();
		  occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
		  double minX, minY, minZ, maxX, maxY, maxZ;
		  octree.getMetricMin(minX, minY, minZ);
		  octree.getMetricMax(maxX, maxY, maxZ);
		  double h = (1.0 - fmin(fmax((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * m_colorFactor;
		  occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
	  }
  } 
  octomap_msgs::binaryMapToMsg(octree, bmap_msg);
  
  fremenCloud.header.frame_id = "/map";
  fremenCloud.header.stamp = ros::Time::now();
  
  octomap_pub_ptr->publish(bmap_msg);
  retrieve_pub_ptr->publish(occupiedNodesVis);
  
  ROS_INFO("Octomap published!");
  
  res.result = true;

  return true;
}

void pose_cb (const geometry_msgs::PosePtr& pose)
{
	if (waitForPose){
		gridPtr->setPose(pose->position.x,pose->position.y);
		ROS_INFO("Position updated %f %f\n",pose->position.x,pose->position.y);
		waitForPose = false;
		waitForMap = true;
	}
}

void name_cb (const std_msgs::StringPtr& input)
{
	currentMap = -1;
	for (int i =0;i<numMaps;i++){
		if (strcmp(input->data.c_str(),gridArray[i]->name)==0) currentMap = i; 
	}
	if (currentMap == -1){
		currentMap = numMaps; 
		gridArray[numMaps++] = new CFremenGrid(DIM_X*DIM_Y*DIM_Z);
		gridArray[currentMap]->setName(input->data.c_str());
		gridPtr = gridArray[currentMap];
	}
	ROS_INFO("Receiving data about map %s, which is map %i of %i total maps. It already has %i observations.\n",gridPtr->name,currentMap,numMaps,gridPtr->signalLength);
	waitForPose = true;
}

int main(int argc,char *argv[])
{
  ros::init(argc, argv, "froctomap");
  ros::NodeHandle n;

  n.param("resolution", resolution, 0.05);
  n.param("colorFactor", m_colorFactor, 0.8);
  n.param<std::string>("filename", filename, "week.bin");

  //Fremen Grid:
  gridPtr = NULL;
  
  //Subscribers:
  ros::Subscriber sub_octo = n.subscribe("/octomap_binary", 1000, octomapCallback);
  ros::Subscriber sub_pose = n.subscribe ("/robot_pose", 1, pose_cb);
  ros::Subscriber sub_name = n.subscribe ("/ptu_sweep/current_node", 1, name_cb);
  
  //Publishers:
  ros::Publisher octomap_pub = n.advertise<octomap_msgs::Octomap>("/froctomap", 100);
  octomap_pub_ptr = &octomap_pub;
  
  ros::Publisher retrieve_pub = n.advertise<visualization_msgs::MarkerArray>("/froctomap_retrieved", 100);
  ros::Publisher estimate_pub = n.advertise<visualization_msgs::MarkerArray>("/froctomap_estimate", 100);
  retrieve_pub_ptr = &retrieve_pub;
  estimate_pub_ptr = &estimate_pub;
  
  //Services:
  ros::ServiceServer retrieve_service = n.advertiseService("retrieve_octomap", retrieve_octomap);
  ros::ServiceServer estimate_service = n.advertiseService("estimate_octomap", estimate_octomap);
  ros::ServiceServer save_service = n.advertiseService("save_grid", save_octomap);
  ros::ServiceServer update_service = n.advertiseService("update_grid", update_octomap);

  ros::spin();
  
  return 0;
}
