#include "ros/ros.h"
#include "fremen/GenerateOctomap.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_octomap");
  
  if (argc != 2){
    ROS_INFO("usage: generate_octomap stamp");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<fremen::GenerateOctomap>("generate_octomap");
  
  fremen::GenerateOctomap srv;
  
  srv.request.stamp = atoi(argv[1]);
  
  if (client.call(srv)){
    if(srv.response.result)
      ROS_INFO("Octomap generated!");
    else
      ROS_INFO("Failed to generate the octomap!");
  }
  else{
    ROS_ERROR("Failed to call service generate_octomap");
    return 1;
  }

  return 0;
}