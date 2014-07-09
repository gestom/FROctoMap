#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <sstream> 

int
main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	std::cout << "Loaded "
		<< cloud.width * cloud.height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;

	std::fstream fs;
	std::stringstream ss;
	fs.open(argv[2], std::fstream::out);
	for (size_t i = 0; i < cloud.points.size (); ++i) fs << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << " " << (int)(cloud.points[i].r) << " " << (int)(cloud.points[i].g) << " " <<(int)(cloud.points[i].b) << "\n";
	fs.close();
	return (0);
} 
