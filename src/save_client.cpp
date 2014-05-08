#include "ros/ros.h"
#include "fremen/SaveGrid.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_grid");
  
  if (argc != 3){
    ROS_INFO("usage: save_grid filename lossy");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<fremen::SaveGrid>("save_grid");
  
  fremen::SaveGrid srv;
  
  srv.request.filename = argv[1];
  srv.request.lossy = atoi(argv[2]);
  
  if (client.call(srv)){
    ROS_INFO("3D Grid Saved (Size: %d)", srv.response.size);
  }else{
    ROS_ERROR("Failed to call service save_grid");
    return 1;
  }

  return 0;
}