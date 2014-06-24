#include "ros/ros.h"
#include "fremen/UpdateGrid.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "update_client");
  
  if (argc != 4){
    ROS_INFO("usage: update_client order precision lossy");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<fremen::UpdateGrid>("update_grid");
  
  fremen::UpdateGrid srv;
  
  srv.request.order = atoi(argv[1]);
  srv.request.precision = atof(argv[2]);
  srv.request.lossy = atoi(argv[3]);
  
  if (client.call(srv)){
    ROS_INFO("3D Grid Updated (Precision: %f | Size: %d)", srv.response.precision, srv.response.size);
  }else{
    ROS_ERROR("Failed to call service update_grid");
    return 1;
  }

  return 0;
}