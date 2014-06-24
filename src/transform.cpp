#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <pcl/filters/voxel_grid.h>
// PCL specific includes
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/passthrough.h>


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

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& inputCloud)
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
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber subcloud = nh.subscribe ("/transform_pc2/depth_registered/points", 1, cloud_cb);
	ros::Subscriber subname = nh.subscribe ("/ptu_sweep/current_node", 1, name_cb);
	ros::Subscriber subpose = nh.subscribe ("/robot_pose", 1, pose_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("/froctomap/localpoints", 1);

	// Spin
	ros::spin ();
}
