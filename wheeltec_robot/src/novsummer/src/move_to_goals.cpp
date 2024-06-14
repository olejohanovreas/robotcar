#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <utility> // for std::pair
#include <geometry_msgs/PoseStamped.h>
#include <amcl/map/map.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <teb_local_planner/FeedbackMsg.h>
#include <teb_local_planner/TrajectoryMsg.h>
#include <teb_local_planner/TrajectoryPointMsg.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <string>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int y;
float powerVoltageIndicator; 
bool charge = false; 
ros::Time start_t; 

void moveGoal(double position_x, double position_y, double orientation_w, MoveBaseClient& ac) {
  // we'll send a goal to the robot
  move_base_msgs::MoveBaseGoal goal;

  // Set the goal target_pose
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = position_x;
  goal.target_pose.pose.position.y = position_y;
  goal.target_pose.pose.orientation.w = orientation_w;

  std::cout << "Goal sent to ROS" << std::endl;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  
/*
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal reached!");
  else
    ROS_INFO("Goal failed, check the costmap for shadow obstacles");
    */
}

std::tuple<double, double, double> chooseOfficeSpace(int officeSpaceIndex) {
  std::vector<std::tuple<double, double, double>> points;

  // Add points to the vector
  // Each point has x, y, and w values
  points.push_back(std::make_tuple(1, 4.4, 1)); // 1
  points.push_back(std::make_tuple(1, 9.9, 1)); // 2
  points.push_back(std::make_tuple(1.5, 18.1, 1)); // 3
  points.push_back(std::make_tuple(0, 0, 1)); // 4
  points.push_back(std::make_tuple(1.55, 23.2, 1)); // 5
  points.push_back(std::make_tuple(1.3, 28.6, 1)); // 6
  points.push_back(std::make_tuple(1.4, 37.2, 1)); // 7
  points.push_back(std::make_tuple(1.66, 43.5, 1)); // 8
  points.push_back(std::make_tuple(7.9, -1.1, 0.5)); // 9
  points.push_back(std::make_tuple(9.75, 4.1, 2)); // 10
  points.push_back(std::make_tuple(9.6, 9.22, 2)); // 11
  points.push_back(std::make_tuple(9.6, 14.5, 2)); // 12
  points.push_back(std::make_tuple(9.6, 19.5, 2)); // 13
  points.push_back(std::make_tuple(9.6, 24.7, 2)); // 14
  points.push_back(std::make_tuple(9.9, 29.7, 2)); // 15
  points.push_back(std::make_tuple(10.4, 35.4, 2)); // 16
  points.push_back(std::make_tuple(10.4, 41.7, 2)); // 17
  points.push_back(std::make_tuple(10.5, 47, 2)); // 18
  points.push_back(std::make_tuple(3.23, 7.25, 1)); // 19

  double x = std::get<0>(points[officeSpaceIndex]);
  double y = std::get<1>(points[officeSpaceIndex]);
  double w = std::get<2>(points[officeSpaceIndex]);
  std::cout << "Coordinates on map " << officeSpaceIndex << ": (" << x << ", " << y << ", " << w << ")" << std::endl;
  return std::make_tuple(x, y, w);

}

std::tuple<double, double, double> positionRobot;

// void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
//   double x = msg->pose.pose.position.x;
//   double y = msg->pose.pose.position.y;
//   double w = tf::getYaw(msg->pose.pose.orientation);

//   //std::cout << "Current robot position in map at " << ros::Time::now() << ": (" << x << ", " << y << ", " << w << ")" << std::endl;

//   positionRobot = std::make_tuple(x, y, w);
// }

std::vector<teb_local_planner::TrajectoryPointMsg> trajectory_est;

bool dynamic_obstacle_detected = false;

void obstacleArrayMsgCallback(const costmap_converter::ObstacleArrayMsg::ConstPtr& data) {
  // Check if the obstacle array is not empty
  if (data->obstacles.size() > 0) {
    const auto& obstacle = data->obstacles[0];
    const auto& velocities = obstacle.velocities.twist;

    // Check if velocities exceed the threshold
    if (velocities.linear.x > 1 || velocities.linear.y > 1) {
      dynamic_obstacle_detected = true;
    } else {
      dynamic_obstacle_detected = false;
    }
  } else {
    dynamic_obstacle_detected = false;
  }
}

void stopAndWait (MoveBaseClient& ac, int savedIndex) {
  ROS_INFO("Dynamic obstacle detected. Stopping...");
  ac.cancelGoal();
  ROS_INFO("Dynamic obstacle detected. Goal canceled");
  bool obstacleCleared = false;
  

  // Wait until the obstacle is no longer detected
  while (!obstacleCleared) {
  ros::spinOnce();
    if (!dynamic_obstacle_detected) {
      obstacleCleared = true;
    }
    ros::Duration(0.5).sleep();
    ROS_INFO("obstacle still not clear");
    }
    ROS_INFO("obstacle cleared replanning");

        /*
        double xpR = std::get<0>(positionRobot);
        double ypR = std::get<1>(positionRobot);
        double wpR = std::get<2>(positionRobot);
        std::cout << "Coordinates for stop : (" << xpR << ", " << ypR << ", " << wpR << ")" << std::endl;
        moveGoal(xpR, ypR, wpR, ac);
        */

        //ros::Duration(10.0).sleep();
  std::tuple<double, double, double> chosenPoint;
  if (savedIndex == 1) {
    chosenPoint = chooseOfficeSpace(0);
  } else {
    chosenPoint = chooseOfficeSpace(savedIndex - 1);
  }
  double x = std::get<0>(chosenPoint);
  double y = std::get<1>(chosenPoint);
  double w = std::get<2>(chosenPoint);
  std::cout << "Previous Goal at index " << savedIndex << ": (" << x << ", " << y << ", " << w << ")" << std::endl;
  moveGoal(x, y, w, ac);
  bool planning = true;
  dynamic_obstacle_detected = false;
}




void websiteCallback(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard: [%s]", msg -> data.c_str()); 
  y = atoi(msg -> data.c_str()); 
  //xx = atoi(msg -> data.c_str()); 
  std::cout << y << std::endl;  
}

void voltageCallback(const std_msgs::Float32::ConstPtr& msg){
  //ROS_INFO("Current voltage: [%f]", msg -> data);
  powerVoltageIndicator = msg -> data; 
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "move_to_goals");
  ros::NodeHandle nh;
  
  //Subscribe to robot position 
  //ros::Subscriber positionSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, positionCallback);

  // Subscribe to obstacle array topic
  ros::Subscriber obstacleSub = nh.subscribe<costmap_converter::ObstacleArrayMsg>("/standalone_converter/costmap_obstacles", 1, obstacleArrayMsgCallback);

  // Subscribe to website publisher
  ros::Subscriber htmlSub = nh.subscribe<std_msgs::String>("/goal_destination", 1, websiteCallback); 

  ros::Subscriber voltageSub = nh.subscribe<std_msgs::Float32>("/PowerVoltage", 1, voltageCallback);

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }


  //startLoop(ac); 
  bool planning = false;
  ros::Rate rate(1.0);

    
  while (ros::ok()) {
    ros::spinOnce();
    int index;
    std::cout << y << std::endl;
    std::cout << powerVoltageIndicator << std::endl;
    
        // Direct the robot to home if power level to low:
    if (powerVoltageIndicator >= 0.1 && powerVoltageIndicator <= 24.5){
      if (powerVoltageIndicator <= 20.5){
        moveGoal(1, 0, 1, ac);
        while(powerVoltageIndicator < 23.5){
          ros::spinOnce();
          std::cout << "Currently charging, please wait" << "Current power level: " << powerVoltageIndicator << std::endl; 
          ros::Duration(10.0).sleep(); 
          if (powerVoltageIndicator >= 23.5){
            break; 
          }
        }
      }    
    } 

    if(planning == false && y>0){
    
    std::cout << "Enter the index (1-4, or 0 to exit, and where 4 is home): ";
    //std::cin >> index;
      index = y; 
      if (index == 0) {
        std::cout << "Exiting..." << std::endl;
        break;
      } else if (index < 1 || index > 19) {
        std::cout << "Invalid index! Please enter an index from 1 to 19." << std::endl;
        continue;
      }
      else{
        std::tuple<double, double, double> chosenPoint;
        if (index == 1) {
          chosenPoint = chooseOfficeSpace(0);
        } else {
          chosenPoint = chooseOfficeSpace(index - 1);
        }

        double x = std::get<0>(chosenPoint);
        double y = std::get<1>(chosenPoint);
        double w = std::get<2>(chosenPoint);
        std::cout << "Coordinates at index " << index << ": (" << x << ", " << y << ", " << w << ")" << std::endl;
        moveGoal(x, y, w, ac);
        planning = true;
      }

    } else if (planning == true){
      std::cout << "currently planning" << std::endl;
      int savedIndex = index;
        if(dynamic_obstacle_detected){
          stopAndWait(ac, savedIndex);
        }


    }
    else{
      std::cout << "Waiting..." << std::endl; 
    }
    

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && dynamic_obstacle_detected == false){
      ROS_INFO("Goal reached!");
    y = 0; 
    planning = false;
    }
    else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
      ROS_INFO("Goal failed, check the costmap for shadow obstacles");
      planning = false;
    }


      rate.sleep();
    
    }
  
return 0;

}
