#include <ros/ros.h>
#include <fremen/froctomapAction.h>
#include <actionlib/server/simple_action_server.h>
#include "fremen/GenerateOctomap.h"
#include "fremen/UpdateGrid.h"
#include "fremen/SaveGrid.h"
#include "fremen/RetrieveOctomap.h"
#include "fremen/EstimateOctomap.h"


ros::ServiceClient *predict_client_ptr, *update_client_ptr, *save_client_ptr, *estimate_client_ptr, *retrieve_client_ptr;

typedef actionlib::SimpleActionServer<fremen::froctomapAction> Server;

void execute(const fremen::froctomapGoalConstPtr& goal, Server* as)
{

  if(goal->name_action == "predict"){
    ROS_INFO("Predicting...!");
    fremen::GenerateOctomap srv;
    srv.request.stamp = goal->stamp;
  
    if(predict_client_ptr->call(srv)){
      if(srv.response.result){
	ROS_INFO("Octomap generated!");
	as->setSucceeded();
      }else{
	ROS_INFO("Failed to generate the octomap!");
	as->setAborted();
      }
    }else{
      ROS_ERROR("Failed to call service generate_octomap");
      as->setAborted();
      }
      
  }else if(goal->name_action == "update"){
    ROS_INFO("Updating...!");
    fremen::UpdateGrid srv;
    srv.request.order = goal->order;
    srv.request.precision = goal->precision;
    srv.request.lossy = goal->lossy;
  
    if(update_client_ptr->call(srv)){
      ROS_INFO("3D Grid Updated (Precision: %f | Size: %d)", srv.response.precision, srv.response.size);
      as->setSucceeded();
    }else{
      ROS_ERROR("Failed to call service update_grid");
      as->setAborted();
    }
    
  }else if(goal->name_action == "save"){
    ROS_INFO("Saving...!");
    fremen::SaveGrid srv;
    srv.request.filename = goal->filename;
    srv.request.lossy = goal->lossy;
  
    if (save_client_ptr->call(srv)){
      ROS_INFO("3D Grid Saved (Size: %d)", srv.response.size);
      as->setSucceeded();
    }else{
      ROS_ERROR("Failed to call service save_grid");
      as->setAborted();
    }
  }else if(goal->name_action == "recover_octomap"){
    ROS_INFO("Recovering...!");
    fremen::RetrieveOctomap srv;
    srv.request.stamp = goal->stamp;
  
    if (retrieve_client_ptr->call(srv)){
      ROS_INFO("Octomap retrieved!");
      as->setSucceeded();
    }else{
      ROS_ERROR("Failed to call service RetrieveOctomap");
      as->setAborted();
    }
  }else if(goal->name_action == "estimate_octomap"){
    ROS_INFO("Estimating...!");
    fremen::EstimateOctomap srv;
    srv.request.stamp = goal->stamp;
    srv.request.minProbability = goal->minprobability;
    srv.request.maxProbability = goal->maxprobability;
  
    if (estimate_client_ptr->call(srv)){
      ROS_INFO("Octomap Estimated");
      as->setSucceeded();
    }else{
      ROS_ERROR("Failed to estimate octomap");
      as->setAborted();
    }
  }else{
    as->setAborted();
  }
    
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
  ros::ServiceClient estimate_client = n.serviceClient<fremen::EstimateOctomap>("estimate_octomap");
  estimate_client_ptr = &estimate_client;
  ros::ServiceClient retrieve_client = n.serviceClient<fremen::RetrieveOctomap>("retrieve_octomap");
  retrieve_client_ptr = &retrieve_client;
  
  
  
  //Server:
  Server server(n, "froctomap_action_server", boost::bind(&execute, _1, &server), false);
  server.start();

  ros::spin();
  
  return 0;
  
}



