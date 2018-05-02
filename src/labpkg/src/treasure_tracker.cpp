#include <ros/ros.h>
#include <vector>
#include <string>
#include <logical_camera_plugin/logicalImage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/impl/utils.h>
#include <algorithm>
#include <iostream>
#include <cmath>

namespace tf2

{
    inline
    void convert(const geometry_msgs::Transform& trans, geometry_msgs::Pose& pose)
    {
        pose.orientation = trans.rotation;
        pose.position.x = trans.translation.x;
        pose.position.y = trans.translation.y;
        pose.position.z = trans.translation.z;
    }

    inline
    void convert(const geometry_msgs::Pose& pose, geometry_msgs::Transform& trans)
      {
        trans.rotation = pose.orientation;
        trans.translation.x = pose.position.x;
        trans.translation.y = pose.position.y;
        trans.translation.z = pose.position.z;
    }

    inline
    void convert(const geometry_msgs::TransformStamped& trans, geometry_msgs::PoseStamped& pose)
    {
        convert(trans.transform, pose.pose);
        pose.header = trans.header;
    }

    inline
    void convert(const geometry_msgs::PoseStamped& pose, geometry_msgs::TransformStamped& trans)
    {
        convert(pose.pose, trans.transform);
        trans.header = pose.header;
    }

	inline
	void merge(const geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pose1, geometry_msgs::PoseStamped &pose3){
		
		pose3.pose.position.x = pose.pose.position.x - pose1.pose.position.x;
		pose3.pose.position.y = pose.pose.position.y - pose1.pose.position.y;
		pose3.pose.position.z = pose.pose.position.z - pose1.pose.position.z;

	}
		
}


std::vector<logical_camera_plugin::logicalImage> storeTreasure;
tf2_ros::Buffer buffer;

void getScan(const logical_camera_plugin::logicalImage &scanMsg){

	
	tf2::Vector3 scanPos = tf2::Vector3(scanMsg.pose_pos_x, scanMsg.pose_pos_y, scanMsg.pose_pos_z);
	tf2::Quaternion scanOrient = tf2::Quaternion(scanMsg.pose_rot_x, scanMsg.pose_rot_y, scanMsg.pose_rot_z, scanMsg.pose_rot_w);

	//START
	geometry_msgs::TransformStamped logicalTrans;
	tf2::convert( tf2::Transform(scanOrient, scanPos), logicalTrans.transform);

	logicalTrans.header.stamp = ros::Time(0);
	logicalTrans.header.frame_id = "logical_camera_link";
	logicalTrans.child_frame_id = scanMsg.modelName;

	geometry_msgs::PoseStamped finalPose;
	geometry_msgs::PoseStamped poseHold;
	geometry_msgs::PoseStamped poseHold1;
	tf2::convert( buffer.transform(logicalTrans, "map", ros::Duration(1.0)), finalPose);
	//END

	/*
	tf2::convert( buffer.lookupTransform("base_link", "map", ros::Time(0)), poseHold1);
	tf2::merge(poseHold, poseHold1, finalPose);*/
	
	bool readyToAdd = true;

	for(int i = 0; i < storeTreasure.size(); i++){

		//if (scanMsg.pose_pos_x == storeTreasure[i].pose_pos_x && scanMsg.pose_pos_y == storeTreasure[i].pose_pos_y && scanMsg.pose_rot_x == storeTreasure[i].pose_rot_x){
		if (scanMsg.modelName == storeTreasure[i].modelName){

			ROS_INFO_STREAM("Did not add repeat Treasure");
			readyToAdd = false;

		}

	}

	std::string tempMsg = scanMsg.modelName;

	if(readyToAdd == true){
	
		if(tempMsg[1] == 'r' && tempMsg[2] == 'e' && tempMsg[3] == 'a'){

			storeTreasure.push_back(scanMsg);
			std::cout << "Added Treasure " << tempMsg << " at " << finalPose;
			ROS_INFO_STREAM(scanMsg);
			
		}

	}

		
}

void print(){

	//size_t i = 0 ??
	for(int i = 0; i < storeTreasure.size(); i++){

		ROS_INFO_STREAM(storeTreasure[i]);

	}

}

int main(int argc, char ** argv){

	ros::init(argc, argv, "treasure_finder");
	ros::NodeHandle nh;

	ros::Subscriber subSensor = nh.subscribe("/objectsDetected", 1000, &getScan);

	tf2_ros::TransformListener tf_listener(buffer, nh);

	//print();

	ros::spin();

}
