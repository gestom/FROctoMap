#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <pcl/filters/voxel_grid.h>
// PCL specific includes
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/passthrough.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h" 
#include <pcl/registration/icp.h>

float x,y,a;
float gridResolution = 0.1;
ros::Publisher pub;
int pcCounter = 0;
pcl::PointCloud<pcl::PointXYZ>::Ptr completeCloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr reference(new pcl::PointCloud<pcl::PointXYZ>);

void pose_cb (const geometry_msgs::PosePtr& pose)
{
	/*if (pcCounter > 0 && pcCounter < 10){
		x = pose->position.x;
		y = pose->position.y;
		printf("Position updated %f %f\n",x,y);
	}*/
}

void name_cb (const std_msgs::StringPtr& input)
{
	//if (input->data == "Aisle2")
	/*if (input->data == "Ramp2")
	{
		printf("Starting to listen\n");
		pcCounter = 0;
	}*/
}


void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg)
{
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	sensor_msgs::PointCloud2 initial2,final;
	pcl::PointCloud<pcl::PointXYZ> registered;
	pcl::PCLPointCloud2 transformed2,output2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr subsampled1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed1(new pcl::PointCloud<pcl::PointXYZ>);

	tf::Matrix3x3 basis(1,0,0,0,1,0,0,0,1);
	tf::Vector3 origin(-2.323683,5.255004,-1.45);
//	tf::Vector3 origin(5.0,5.0,-1.45);
	tf::Transform trf(basis,origin);
	pcl_ros::transformPointCloud("/ptu_pan_motor",trf,*msg,initial2);
	pcl_conversions::toPCL(initial2, transformed2);
//	pcl_conversions::toPCL(*msg, transformed2);
	pcl::fromPCLPointCloud2(transformed2, *transformed1);

	/*subsample*/
	grid.setLeafSize (0.01, 0.01, 0.01);
	grid.setInputCloud (transformed1);
	grid.filter (*subsampled1); 

	if (pcCounter == 0)
	{
		*reference = *subsampled1;
		pcCounter = 1;
		pcl::toPCLPointCloud2(*subsampled1,output2);
		printf("Reference scan created!\n");
	}else{
		/*register*/
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setEuclideanFitnessEpsilon (1);
		icp.setInputCloud(subsampled1);
		icp.setInputTarget(reference);
		icp.align(registered);
		printf("Aligning!\n");
		pcl::toPCLPointCloud2(registered,output2);
	}

	output2.header.frame_id = "/ptu_pan_motor";
	//output2.header.frame_id = "/map";
	pub.publish(output2);
} 

int main (int argc, char** argv)
{
	x=y=a=0;
	// Initialize ROS
	ros::init (argc, argv, "transform");
	ros::NodeHandle nh;

	printf("HOVADO!\n");
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber subcloud = nh.subscribe ("/local_metric_map/merged_point_cloud", 1, cloud_cb);
//	ros::Subscriber subname = nh.subscribe ("/ptu_sweep/current_node", 1, name_cb);
//	ros::Subscriber subpose = nh.subscribe ("/robot_pose", 1, pose_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("/froctomap/localpoints", 1);

	// Spin
	ros::spin ();
}
