#include <ros/ros.h>
#include <fremen/froctomapAction.h>
#include <actionlib/server/simple_action_server.h>
#include "fremen/GenerateOctomap.h"
#include "fremen/UpdateGrid.h"
#include "fremen/SaveGrid.h"


ros::ServiceClient *predict_client_ptr, *update_client_ptr, *save_client_ptr;

typedef actionlib::SimpleActionServer<fremen::froctomapAction> Server;

void execute(const fremen::froctomapGoalConstPtr& goal, Server* as)
{

  if(goal->name_action == "predict"){
    
    fremen::GenerateOctomap srv;
    srv.request.stamp = goal->stamp;
  
    if(predict_client_ptr->call(srv)){
      if(srv.response.result)
	ROS_INFO("Octomap generated!");
      else
	ROS_INFO("Failed to generate the octomap!");
    }else{
      ROS_ERROR("Failed to call service generate_octomap");
      }
      
  }else if(goal->name_action == "update"){
    
    fremen::UpdateGrid srv;
    srv.request.order = goal->order;
    srv.request.precision = goal->precision;
    srv.request.lossy = goal->lossy;
  
    if(update_client_ptr->call(srv)){
      ROS_INFO("3D Grid Updated (Precision: %f | Size: %d)", srv.response.precision, srv.response.size);
    }else{
      ROS_ERROR("Failed to call service update_grid");
    }
    
  }else if(goal->name_action == "save"){
    
    fremen::SaveGrid srv;
    srv.request.filename = goal->filename;
    srv.request.lossy = goal->lossy;
  
    if (save_client_ptr->call(srv)){
      ROS_INFO("3D Grid Saved (Size: %d)", srv.response.size);
    }else{
      ROS_ERROR("Failed to call service save_grid");
    }
    
  }else{
    ROS_INFO("error!");
    as->setSucceeded();
  }
    
  as->setSucceeded();
}


int main(int argc,char *argv[])
{
  
  ros::init(argc, argv, "froctomap_as_server");
  ros::NodeHandle n;
  
  //Services:
  ros::ServiceClient predict_client = n.serviceClient<fremen::GenerateOctomap>("generate_octomap");
  predict_client_ptr = &predict_client;
  ros::ServiceClient update_client = n.serviceClient<fremen::UpdateGrid>("update_grid");
  update_client_ptr = &update_client;
  ros::ServiceClient save_client = n.serviceClient<fremen::SaveGrid>("save_grid");
  save_client_ptr = &save_client;
  
  //Server:
  Server server(n, "froctomap_action_server", boost::bind(&execute, _1, &server), false);
  server.start();
  
  


  ros::spin();
  
  return 0;
  
}



