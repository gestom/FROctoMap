#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "CFremenGrid.h"
#include "CTimer.h"

using namespace std;

CFremenGrid *grid_ptr;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){

	//receive pose and add the height of the robot
	geometry_msgs::Point robot_head;
	robot_head.x = msg	
	

	//ray casting (FoV)

	//gain for each cell in the ground floor

}

int main(int argc,char *argv[]){
  
  	ros::init(argc, argv, "froctomap_gain");
  	ros::NodeHandle n;

  	n.param<std::string>("filename", filename, "week.bin");
	
	//Fremen Grid:
	load("week7_grid");
	grid_ptr = &grid;
  	ros::spin();
  
  	return 0;
  
}



