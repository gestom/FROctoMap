#include <ros/ros.h>
#include <fremen/froctomapAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<fremen::froctomapAction> Client;


int main(int argc,char *argv[])
{
  
  ros::init(argc, argv, "froctomap_as_client");
  
  if (argc != 7){
    ROS_INFO("usage: froctomap_ActionClient action stamp filename order precision lossy");
    return 1;
  }
  
  
  Client client("froctomap_action_server", true); // true -> don't need ros::spin()
  client.waitForServer();
  
  
  fremen::froctomapGoal goal;
  
  goal.name_action = argv[1];
  goal.stamp = atoi(argv[2]);
  goal.filename = argv[3];
  goal.order = atoi(argv[4]);
  goal.precision = atof(argv[5]);
  goal.lossy = atoi(argv[6]);
  
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(1.0));
  
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("%s action done!!!", argv[1]);
  else
    ROS_ERROR("Action failed!!!");
  
  return 0;
  
}



