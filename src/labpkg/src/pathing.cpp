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
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>

using namespace std;
bool shouldStop = false;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;

int gridMap[20][20];
int nextX = 0;
int nextY = 0;
geometry_msgs::Pose2D targetPose;
geometry_msgs::TransformStamped currentPos;
tf2_ros::Buffer buffer;
bool allDone = false;
bool isStuck = false;

void floodFillInit(){

	for(int i = 0; i < 20; i++){

		for(int j = 0; j < 20; j++){

			gridMap[i][j] = 999;

		}

	}

}

void floodFillPlanner(int arr[20][20], int x, int y){

	currentPos = buffer.lookupTransform("husky_alpha/base_link", "odom", ros::Time(0));

	int count = 0;

	for(int i = 0; i < 20; i++){

		for(int j = 0; j < 20; j++){

			if(arr[i][j] == 0){

				count++;

				if(count == 400){

					ac->cancelAllGoals();
					allDone = true;
			
				}

			}

		}

	}

	arr[x][y] = 0;

	if(arr[x + 1][y] == 0){

		//floodFillPlanner(arr, x + 1, y);
		nextX = x + 1;
		nextY = y;
		targetPose.x += 1;
		targetPose.y = currentPos.transform.translation.y;
		
	}else if(arr[x][y + 1] == 0){

		//floodFillPlanner(arr, x, y + 1);
		nextX = x;
		nextY = y + 1;
		targetPose.x = currentPos.transform.translation.x;
		targetPose.y += 1;

	}else if(arr[x - 1][y] == 0){

		//floodFillPlanner(arr, x - 1, y);
		nextX = x - 1;
		nextY = y;
		targetPose.x -= 1;
		targetPose.y = currentPos.transform.translation.y;

	}else if(arr[x][y - 1] == 0){

		//floodFillPlanner(arr, x, y - 1);
		nextX = x;
		nextY = y - 1;
		targetPose.x = currentPos.transform.translation.x;
		targetPose.y -= 1;

	}else{

		for(int i = 0; i < 20; i++){

			if(arr[x + i][y] == 0){

				//floodFillPlanner(arr, x + i, y);
				nextX = x + i;
				nextY = y;
				targetPose.x += i;
				targetPose.y = currentPos.transform.translation.y;

			}else if(arr[x][y + i] == 0){

				//floodFillPlanner(arr, x, y + i);
				nextX = x;				
				nextY = y + i;
				targetPose.x = currentPos.transform.translation.x;
				targetPose.y += i;

			}else if(arr[x - i][y] == 0){

				//floodFillPlanner(arr, x - i, y);
				nextX = x - i;
				nextY = y;
				targetPose.x -= i;
				targetPose.y = currentPos.transform.translation.y;

			}else if(arr[x][y - i] == 0){

				//floodFillPlanner(arr, x, y - i);
				nextX = x;
				nextY = y - i;
				targetPose.x = currentPos.transform.translation.x;
				targetPose.y -= i;

			}
		}
	
	}

}
		
void markOnMap(){

	currentPos = buffer.lookupTransform("husky_alpha/base_link", "odom", ros::Time(0));
	float tempX = currentPos.transform.translation.x;
	float tempY = currentPos.transform.translation.y;

	int intX = round(tempX);
	int intY = round(tempY);

	gridMap[intX][intY] = 0;
	gridMap[intX + 1][intY + 1] = 0;
	//gridMap[intX + 2][intY + 2] = 0;
	gridMap[intX - 1][intY] = 0;
	gridMap[intX][intY - 1] = 0;
	gridMap[intX - 1][intY = 1] = 0;


}

void sendNewGoal();
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
    goal.target_pose.pose.position.x = targetPose.x;
    goal.target_pose.pose.position.y = targetPose.y;
    goal.target_pose.pose.orientation.w = 1.0;
    //ROS_INFO_STREAM("Sending new goal: (" << goal.target_pose.pose.position.x << ", " << goal.target_pose.pose.position.y << ")\n");
    ros::Duration(0.1).sleep();
    ac->sendGoal(goal, &serviceDone, &serviceActivated, &serviceFeedback);
}

void updateScan(sensor_msgs::LaserScan scan) {
	for (int i = 0; i < scan.ranges.size(); i++) {
		if (scan.ranges[i] < 0.2) {
			isStuck = true;
			markOnMap();
			ac->cancelAllGoals();
		}
	}
}

int main(int argc,char **argv) {
	ros::init(argc,argv,"saferandomwalk");
    ros::NodeHandle nh;
	tf2_ros::TransformListener listener(buffer);
	ros::Subscriber scanSub = nh.subscribe("/scan", 1000, &updateScan);
	
    ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true);
	ros::Publisher pubTwist = nh.advertise<geometry_msgs::Twist>("husky_alpha/husky_velocity_controller/cmd_vel", 1000);
	floodFillInit();
	ROS_INFO_STREAM("Built Grid Map");
    ROS_INFO_STREAM("Waiting for server to be available...");
    while (!ac->waitForServer()) {
    }
    ROS_INFO_STREAM("done!");
    ac->cancelAllGoals();

	while(!allDone){

		if(!isStuck){
		  
			ros::spinOnce();
			ROS_INFO_STREAM("Moving towards normal goals");
			ac->cancelAllGoals();
			floodFillPlanner(gridMap, nextX, nextY);
    			sendNewGoal();

				ROS_INFO_STREAM("Got to goal, spinning");
				geometry_msgs::Twist targetTwist;
				targetTwist.angular.z = 1.57;
				pubTwist.publish(targetTwist);
    
   		 //ros::spinOnce();

		}else{

			ROS_INFO_STREAM("Stuck, spin once, then going back");
			geometry_msgs::Twist targetTwist;
			targetTwist.angular.z = 1.57;
			pubTwist.publish(targetTwist);

			geometry_msgs::Twist unstuckTwist;
			unstuckTwist.linear.x = -10.0;
			unstuckTwist.linear.y = -10.0;
			pubTwist.publish(unstuckTwist);
			isStuck = false;

		}
	}

	return 0; 
}

//create gridMap
//produce next free coords using floodfill
//send to actionclient to plan
//if run into objec mark on gridmap as traveled to already
//when reaching destination given by gridmap spin once in a circle then get new goal
//gridmap selects the next avaliable slot in the 4 main directions, if it can not find one it will select from increasing values in either direction
//gridmap terminates when all grid areas have been explored (marked with 0)
