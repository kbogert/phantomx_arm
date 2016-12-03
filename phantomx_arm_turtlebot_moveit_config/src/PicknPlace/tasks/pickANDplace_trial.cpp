#include "../nodes/pickANDplace.h"

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

using namespace std;
using namespace picknplace;

int main(int argc, char **argv)
{
	ros::init (argc, argv, "pickANDplace");
	ros::NodeHandle node_handle("~");

	pickANDplace test;

	double msg = -1.5;
	
	//std::vector<double> pickup_coordinates;
	test.rest_coordinates.push_back(0.067);
	test.rest_coordinates.push_back(0.0);
	test.rest_coordinates.push_back(0.82);

	test.pickup_coordinates.push_back(0.330);
	test.pickup_coordinates.push_back(0.001);
	test.pickup_coordinates.push_back(0.435);

	test.place_coordinates.push_back(0.330);
	test.place_coordinates.push_back(0.001);
	test.place_coordinates.push_back(0.435);

	for(int i=0;i<5;i++)
	{
		/*test.execution = test.execute(test.getPickUpLoc(), node_handle);
		test.pick(node_handle);
		if(test.execution == false)
			ROS_INFO_STREAM("\nIK TESTINGGGGG: " << test.execution << "\n");
		msg = -1.5;
		test.grasp(node_handle, msg);*/

		test.rest_coordinates.clear();
		//std::vector<double> rest_coordinates;
		test.rest_coordinates.push_back(0.125);
		test.rest_coordinates.push_back(0.001);
		test.rest_coordinates.push_back(0.657);

		test.execution = test.execute(test.getRestPosition(), node_handle);
		test.pick(node_handle);
		if(test.execution == false)
			ROS_INFO_STREAM("\nIK TESTINGGGGG: " << test.execution << "\n");

		test.execution = test.execute(test.getPlaceLoc(), node_handle);
		test.pick(node_handle);
		if(test.execution == false)
			ROS_INFO_STREAM("\nIK TESTINGGGGG: " << test.execution << "\n");
		msg = 0.0;
		//test.grasp(node_handle, msg);

		test.execution = test.execute(test.getRestPosition(), node_handle);
		test.pick(node_handle);
		if(test.execution == false)
			ROS_INFO_STREAM("\nIK TESTINGGGGG: " << test.execution << "\n");

		/*test.execution = test.execute(test.getPickUpLoc(), node_handle);
		test.pick(node_handle);
		if(test.execution == false)
			ROS_INFO_STREAM("\nIK TESTINGGGGG: " << test.execution << "\n");*/

		test.execution = true;
	}

	return 0;	
}
