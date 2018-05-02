#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <stdlib.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;
bool shouldStop = false;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;
void sendNewGoal();

int gridMap[20][20];
geometry_msgs::Pose2D targetPose;

void floodFillInit(){

	for(int i = 0; i < 20; i++){

		for(int j = 0; i < 20; j++){

			gridMap[i][j] = 999;

		}

	}

}

void floodFillPlanner(int arr[][10], int x, int y){

	//if visited every spot stop
	int count = 0;

	for(int i = 0; i < 20; i++){

		for(int j = 0; j < 20; j++){

			if(arr[i][j] == 0){

				count++;

			}

		}

	}

	if(count == 400){
		
		ROS_INFO_STREAM("Done");
		ac->cancelAllGoals();

	}
	//set current location to 0
	arr[x][y] = 0;


	if(arr[x + 1][y] == 999){

		floodFillPlanner(arr, x + 1, y);
		targetPose.x += 1;
	
	}else if(arr[x][y + 1] == 999){

		floodFillPlanner(arr, x, y + 1);
		targetPose.y += 1;

	}else if(arr[x - 1][y] == 999){

		floodFillPlanner(arr, x - 1, y);
		targetPose.x -= 1;

	}else if(arr[x][y - 1] == 999){

		floodFillPlanner(arr, x, y - 1);
		targetPose.y -= 1;

	}

	
}	


	
void serviceActivated() {}
void serviceFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb) {
}

void serviceDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
	//ROS_INFO_STREAM("Service ended:  " << state.toString().c_str());
	sendNewGoal();
}

void sendNewGoal () {
	move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    int targetPos = (rand() % 14) - 7;   
    goal.target_pose.pose.position.x = targetPose;
    goal.target_pose.pose.position.y = targetPose;
    goal.target_pose.pose.orientation.w = 1.0;
    //ROS_INFO_STREAM("Sending new goal: (" << goal.target_pose.pose.position.x << ", " << goal.target_pose.pose.position.y << ")\n");
    ros::Duration(0.1).sleep();
    ac->sendGoal(goal, &serviceDone, &serviceActivated, &serviceFeedback);
}

void updateScan(sensor_msgs::LaserScan scan) {
	for (int i = 0; i < scan.ranges.size(); i++) {
		if (scan.ranges[i] < 0.2) {
			ac->cancelAllGoals();
		}
	}
}

int main(int argc,char **argv) {
	ros::init(argc,argv,"saferandomwalk");
    ros::NodeHandle nh;
	ros::Subscriber scanSub = nh.subscribe("/scan", 1000, &updateScan);
	//ros::Subscriber subLoc = nh.subscribe("/tf", 1000, &currentLoc);
	
    ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true);
	
    //ROS_INFO_STREAM("Waiting for server to be available...");
    while (!ac->waitForServer()) {
    }
    //ROS_INFO_STREAM("done!");
    ac->cancelAllGoals();
    sendNewGoal();
    
    ros::spin();
	
	return 0; 
}
