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


float x,y,a;
float gridResolution = 0.1;
ros::Publisher pub;
int pcCounter = 0;
pcl::PointCloud<pcl::PointXYZ>::Ptr completeCloud(new pcl::PointCloud<pcl::PointXYZ>);

void pose_cb (const geometry_msgs::PosePtr& pose)
{
	if (pcCounter > 0 && pcCounter < 10){
		x = pose->position.x;
		y = pose->position.y;
		printf("Position updated %f %f\n",x,y);
	}
}

void name_cb (const std_msgs::StringPtr& input)
{
	//if (input->data == "Aisle2")
	if (input->data == "Ramp2")
	{
		printf("Starting to listen\n");
		pcCounter = 0;
	}
}


void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg)
{
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	sensor_msgs::PointCloud2 initial2,final;
	pcl::PCLPointCloud2 transformed2,output2;
	pcl::PointCloud<pcl::PointXYZ> subsampled1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed1(new pcl::PointCloud<pcl::PointXYZ>);

	tf::Matrix3x3 basis(1,0,0,0,1,0,0,0,1);
	tf::Vector3 origin(-2.323683,5.255004,-1.45);
	tf::Transform trf(basis,origin);
	pcl_ros::transformPointCloud("/ptu_pan_motor",trf,*msg,initial2);
	pcl_conversions::toPCL(initial2, transformed2);
	pcl::fromPCLPointCloud2(transformed2, *transformed1);

	/*subsample*/
	grid.setLeafSize (0.05, 0.05, 0.05);
	grid.setInputCloud (transformed1);
	grid.filter (subsampled1); 

	/*register*/



	pcl::toPCLPointCloud2(subsampled1,output2);

	output2.header.frame_id = "/ptu_pan_motor";
	printf("TRANSFORMING!\n");
	pub.publish(output2);
} 

void cloud_cb_sracka(const sensor_msgs::PointCloud2ConstPtr& inputMessage)
{
	pcl::PCLPointCloud2 inputCloud2;
	pcl::PCLPointCloud2 outputCloud2;
	Eigen::Matrix4f scale;
	scale << 1,0,0,0,
	      0,1,0,0,
	      0,0,1,0,
	      0,0,0,1;
	pcl::PointCloud<pcl::PointXYZ> inputCloud1;
	pcl::PointCloud<pcl::PointXYZ> outputCloud1;
	pcl_conversions::toPCL(*inputMessage, inputCloud2);
	pcl::fromPCLPointCloud2(inputCloud2, inputCloud1);
	std::cout << completeCloud->size() << std::endl;
	tf::TransformListener tl;
	tf::StampedTransform trf;
	pcl_ros::transformPointCloud("/head_xtion_rgb_optical_frame",inputCloud1,outputCloud1, tl);

/*	if (tl.waitForTransform("/head_xtion_rgb_optical_frame", inputCloud1.header.frame_id,inputMessage->header.stamp, ros::Duration(20.0) ))
	{
			tl.lookupTransform("/head_xtion_rgb_optical_frame",inputCloud1.header.frame_id,inputMessage->header.stamp, trf);
			pcl_ros::transformPointCloud("/head_xtion_rgb_optical_frame",inputCloud1,outputCloud1, tl);
			//tl.transformPointCloud("/head_xtion_rgb_optical_frame", inputCloud1, outputCloud1);	
			//	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
			/*voxel_grid.setInputCloud (completeCloud);
			  voxel_grid.setLeafSize (0.1, 0.1, 0.1);
			  voxel_grid.filter (cloud1);
			  pcl::transformPointCloud (cloud1, output1,scale);*/

			/*		pcl::toPCLPointCloud2(*completeCloud, output2);
					output2.header.frame_id = "/head_xtion_rgb_optical_frame";
					pub.publish (output2);*/
			//	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;

			/*	voxel_grid.setInputCloud (inputCloud1);
				voxel_grid.setLeafSize (0.05, 0.05, 0.05);
				voxel_grid.filter (outputCloud1);
			printf("Transform OK\n");
			pcl::toPCLPointCloud2(outputCloud1,outputCloud2);
			outputCloud2.header.frame_id = "/map";
			pub.publish (outputCloud2);
			}else{*/
	printf("Timeout when waiting for a transform\n");
	pcl::toPCLPointCloud2(inputCloud1,outputCloud2);
	outputCloud2.header.frame_id = "/map";
	pub.publish (outputCloud2);
}



void cloud_cb_fuck (const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
	if (pcCounter++ < 67){
		pcl::PCLPointCloud2 cloud2;
		pcl::PCLPointCloud2 output2;
		Eigen::Matrix4f scale;
		scale << 1,0,0,0,
		      0,1,0,0,
		      0,0,1,0,
		      0,0,0,1;
		pcl::PointCloud<pcl::PointXYZ> cloud1;
		pcl::PointCloud<pcl::PointXYZ> output1;
		pcl_conversions::toPCL(*inputCloud, cloud2);
		pcl::fromPCLPointCloud2(cloud2, cloud1);
		*completeCloud = cloud1;
		std::cout << completeCloud->size() << std::endl; 
			
		pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
		/*voxel_grid.setInputCloud (completeCloud);
		voxel_grid.setLeafSize (0.1, 0.1, 0.1);
		voxel_grid.filter (cloud1);
		pcl::transformPointCloud (cloud1, output1,scale);*/

/*		pcl::toPCLPointCloud2(*completeCloud, output2);
		output2.header.frame_id = "/head_xtion_rgb_optical_frame";
		pub.publish (output2);*/
		if (pcCounter == 67){
			pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
			voxel_grid.setInputCloud (completeCloud);
			voxel_grid.setLeafSize (0.05, 0.05, 0.05);
			voxel_grid.filter (cloud1);
			pcl::transformPointCloud (cloud1, output1,scale);
			pcl::toPCLPointCloud2(output1, output2);
			output2.header.frame_id = "/map";
			pub.publish (output2);
		}
	}
	printf("%i\n",pcCounter);
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
