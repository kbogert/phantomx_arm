#include <string>

#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>

jointStateSubscriber::jointStateSubscriber(ros::NodeHandle node_handle)
{
	ROS_INFO("Creating Joint States subscriber Object.. \n");
	jointState_topic = "/joint_states";

	joints = node_handle.subscribe(jointState_topic, 1000, &jointStateSubscriber::getJointStates, this);
}

void jointStateSubscriber::getJointStates(const sensor_msgs::JointStatePtr& msg)
{
	if(flag == true)
	{
		for(int jointList=0; jointList < msg->name.size(); jointList++)
		{
			curr_joint_values[msg->name[jointList]] = msg->position[jointList];
			ROS_INFO("Receiving [Joint] %s -> %f ", msg->name[jointList].c_str(), msg->position[jointList]);
		}
		flag = false;
	}

	return;
}
