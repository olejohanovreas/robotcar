#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <utility> // for std::pair

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void moveGoal(double position_x, double orientation_w) {
  //we'll send a goal to the robot
  move_base_msgs::MoveBaseGoal goal;

  // Set the goal target_pose
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = position_x;
  goal.target_pose.pose.orientation.w = orientation_w;

  cout << "Goal sendt to ROS";

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();


  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal reached!");
  else
    ROS_INFO("Goal failed, check the costmap for shadow obstacles");

  };

std::pair<double, double> chooseOfficeSpace(int officeSpaceIndex){
    std::vector<std::pair<double, double>> points;

    // Add points to the vector
    // Where the first is X position and second is orientation of the robot
    points.push_back(std::make_pair(1.0, 1.0));
    points.push_back(std::make_pair(2.0, 1.0));
    points.push_back(std::make_pair(3.0, 1.0));

    if (officeSpaceIndex >= 0 && officeSpaceIndex < points.size()) {
        double x = points[officeSpaceIndex].first;
        double y = points[officeSpaceIndex].second;
        std::cout << "Coordinates on map " << officeSpaceIndex << ": (" << x << ", " << y << ")" << std::endl;
        return std::make_pair(x, y);
    } else {
        std::cout << "Invalid index!" << std::endl;
        return std::make_pair(-1.0, -1.0);
    }
}



int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  int index = 0;

  std::pair<double, double> chosenPoint = chooseOfficeSpace(index);

  if (chosenPoint.first == -1.0 && chosenPoint.second == -1.0) {
    std::cout << "Invalid index!" << std::endl;
  } else {
    std::cout << "Coordinates at index " << index << ": (" << chosenPoint.first << ", " << chosenPoint.second << ")" << std::endl;
    moveGoal(chosenPoint.first, chosenPoint.second);
  }

  

  

  return 0;
}