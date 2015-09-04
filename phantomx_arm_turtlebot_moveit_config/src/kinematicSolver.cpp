#include <ros/ros.h>
#include <string>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>

#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

using namespace std;

std::map <string, double> curr_joint_values;
bool flag = true;

void getJointStates(const sensor_msgs::JointStatePtr& msg)
{
	if(flag == true)
	{
		for(int jointList=0; jointList < msg->name.size(); jointList++)
		{
			curr_joint_values[msg->name[jointList]] = msg->position[jointList];
			ROS_INFO("RECEIVE %s -> %f ", msg->name[jointList].c_str(), msg->position[jointList]);
		}
		flag = false;
	}

	return;
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "kinematicSolver");
	ros::NodeHandle node_handle("~");
	ros::Publisher arm_publisher = node_handle.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal", 1, true);

	ros::Subscriber joints = node_handle.subscribe("/joint_states", 1000, getJointStates);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::WallDuration sleep_time(2.0);
	sleep_time.sleep();

	ROS_INFO("Model frame: ");

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
 	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

	//kinematic_state->setToDefaultValues();
	
		kinematic_state->setJointPositions("joint_1", & curr_joint_values["joint_1"]);
		kinematic_state->setJointPositions("joint_2", & curr_joint_values["joint_2"]);
		kinematic_state->setJointPositions("joint_3", & curr_joint_values["joint_3"]);
		kinematic_state->setJointPositions("joint_4", & curr_joint_values["joint_4"]);

	ROS_INFO("Joint_1 %f", curr_joint_values["joint_1"]);

	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

	// Get Joint Values
	// ^^^^^^^^^^^^^^^^
	// We can retreive the current set of joint values stored in the state for the arm.
	std::vector<double> joint_values;
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	for(std::size_t i = 0; i < joint_names.size(); ++i)
	{
		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	
	control_msgs::FollowJointTrajectoryActionGoal action;

	trajectory_msgs::JointTrajectoryPoint currPoint;
	currPoint.time_from_start = ros::Duration(0.0);

	ros::Time begin = ros::Time::now();

	action.header.stamp = begin;
	action.goal_id.stamp = begin;
//	action.goal.trajectory.header.stamp = begin;
//	action.goal.trajectory.header.frame_id = "/odom/base_footprint/base_link/plate_top_link/arm_base_link";
	action.goal.trajectory.header.frame_id = "/odom";

	//std::string timeString = std::to_string(begin.toSec());


	action.goal_id.id = "/move_arm-1-";

	for(std::size_t i=0; i < joint_names.size()-2; ++i)
	{
		// ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
		currPoint.positions.push_back(joint_values[i]);
//		currPoint.velocities.push_back(0);
//		currPoint.accelerations.push_back(1);

		action.goal.trajectory.joint_names.push_back(joint_names[i+1]);
	}


	action.goal.trajectory.points.push_back(currPoint);

	// Joint Limits
	// ^^^^^^^^^^^^
	// setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
	/* Set one joint in the right arm outside its joint limit */
	joint_values[0] = 1.57;
	kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

	/* Check whether any joint is outside its joint limits */
	ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
	
	/* Enforce the joint limits for this state and check again*/
	kinematic_state->enforceBounds();
	ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

/*double zero = 0.0;

		kinematic_state->setJointPositions("joint_1", &zero);
		kinematic_state->setJointPositions("joint_2", &zero);
		kinematic_state->setJointPositions("joint_3", &zero);
		kinematic_state->setJointPositions("joint_4", &zero);
*/
	kinematic_state->setToRandomPositions(joint_model_group);
//	const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("link_5");

	Eigen::Affine3d end_effector_state = Eigen::Transform<double,3,Eigen::Affine>::Identity();
//	end_effector_state *= Eigen::Translation<double,3>(0.08,0.0,0.82);
	end_effector_state *= Eigen::Translation<double,3>(0.08, 0.0, 0.82);

	geometry_msgs::PoseStamped pose_1;
	geometry_msgs::Pose pose;
	
	pose.position.x = 0.0;
	pose.position.y = 0.0;
	pose.position.z = 0.0;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.0;
	pose.orientation.w = 0.0;

	geometry_msgs::Pose &pose_2 = pose;

	tf::poseEigenToMsg(end_effector_state, pose_2);

	/* Print end-effector pose. Remember that this is in the model frame */
	ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
	ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

	ROS_INFO_STREAM("Translation X: " << pose.position.x);
	ROS_INFO_STREAM("Translation Y: " << pose.position.y);
	ROS_INFO_STREAM("Translation Z: " << pose.position.z);

	ROS_INFO_STREAM("Rotation W: " << pose.orientation.w);
	ROS_INFO_STREAM("Rotation X: " << pose.orientation.x);
	ROS_INFO_STREAM("Rotation Y: " << pose.orientation.y);
	ROS_INFO_STREAM("Rotation Z: " << pose.orientation.z);


	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

	// Now, we can print out the IK solution (if found):
	if (found_ik)
	{
		ROS_INFO("FOUND IK SOLUTION YAYYYY!! ");

		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		for(std::size_t i=0; i < joint_names.size(); ++i)
		{
			ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
		}
			// Get the Jacobian
		// ^^^^^^^^^^^^^^^^
		// We can also get the Jacobian from the :moveit_core:`RobotState`.
		Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
		Eigen::MatrixXd jacobian;

		kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),reference_point_position,jacobian);
		ROS_INFO_STREAM("Jacobian: " << jacobian);

		trajectory_msgs::JointTrajectoryPoint point;

		point.time_from_start = ros::Duration(1.0);


		for(std::size_t i=0; i < joint_names.size()-2; ++i)
			{
				// ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
				point.positions.push_back(joint_values[i]);
//				point.velocities.push_back(1);
//				point.accelerations.push_back(1);

			}

			action.goal.trajectory.points.push_back(point);

		sleep_time.sleep();

		arm_publisher.publish(action);

	}
	else
	{
		ROS_INFO("Did not find IK solution");
	}


	sleep_time.sleep();

	// END_TUTORIAL
	ros::shutdown();

	return 0;
}
