#include "pickANDplace.h"

#include <string>

#include <ros/ros.h>
#include "std_msgs/Float64.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>

using namespace picknplace;

pickANDplace::pickANDplace()
{
	ROS_INFO("Creating Pick and Place Object.. \n");

	referenceFrame = "/odom";
	armPublisher_topic = "/Maulesh/phantomx_arm/arm_controller/follow_joint_trajectory/goal";
	gripperPublisher_topic = "/Maulesh/phantomx_arm/joint_5/command";
	jointState_topic = "/Maulesh/phantomx_arm/joint_states";

	flag = true;
	execution = true;
	max_trial = 5;
	isSimulation = false;
}

pickANDplace::~pickANDplace()
{
	ROS_INFO("Deleting Pick and Place Object.. \n");
}

bool pickANDplace::checkIK(std::vector<double> goalPose, ros::NodeHandle node_handle, int count)
{
	ros::WallDuration sleep_time(2.0);
	sleep_time.sleep();

	// Load the robot_description model onto the param server
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
 	
 	// Create a holder for the Kinematic Model
 	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model Frame: %s", kinematic_model->getModelFrame().c_str());

	// Create a handle for the Robot State
	// Representation of a robot's state. This includes position, velocity, acceleration and effort.
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

	// Set the Joint positions for our kinematic state
	kinematic_state->setJointPositions("joint_1", & curr_joint_values["joint_1"]);
	kinematic_state->setJointPositions("joint_2", & curr_joint_values["joint_2"]);
	kinematic_state->setJointPositions("joint_3", & curr_joint_values["joint_3"]);
	kinematic_state->setJointPositions("joint_4", & curr_joint_values["joint_4"]);

	// Define the Joint Model group for the 'arm' and store the Joints[_name et all] 
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

	// Get Joint Values
	// Retreive the current set of joint values stored in the kinematic [robot]state for the arm
	std::vector<double> joint_values;
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

	// Define a JointTrajectoryPoint to be sent as an action goal for the IK
	trajectory_msgs::JointTrajectoryPoint currPoint;
	currPoint.time_from_start = ros::Duration(0.1);

	// Begin ROS Time
	ros::Time begin = ros::Time::now();

	// Fill in the details about the FollowJointTrajectoryActionGoal which will be used by the IK Solver
	action.header.stamp = begin;
	action.goal_id.stamp = begin;
	//action.goal_id.id = "/move_phantomx_arm-";
	double sec = begin.sec + begin.nsec/10000000.0;
	
	char buffer[256];
	std::snprintf(buffer, sizeof(buffer), "/move_phantomx_arm-%g", sec);
	
	action.goal_id.id = string(buffer);
	action.goal.trajectory.header.frame_id = referenceFrame;

	// Clear past Action Goals from the vector
	action.goal.trajectory.points.clear();
	action.goal.trajectory.joint_names.clear();

	for(std::size_t i=0; i < joint_names.size()-2; ++i)
	{
		currPoint.positions.push_back(joint_values[i]);
		/* Optionally set Velocities and Accelarations for the Motion Planing
		point.velocities.push_back(1);
		point.accelerations.push_back(1); */

		action.goal.trajectory.joint_names.push_back(joint_names[i+1]);
	}
	action.goal.trajectory.points.push_back(currPoint);

	/* Joint Limits
	   setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
	   Set one joint in the right arm outside its joint limit */
	joint_values[0] = 1.57;
	kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

	// Check whether any joint is outside its joint limits 
	ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
	
	// Enforce the joint limits for this state and check again
	kinematic_state->enforceBounds();
	ROS_INFO_STREAM("Current state [after enforcing bounds] is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

	// Set the Robot State to random positions in the joint model group
	kinematic_state->setToRandomPositions(joint_model_group);

	// Define the end_effector_state as an Eigen Matrix
	Eigen::Affine3d end_effector_state = Eigen::Transform<double,3,Eigen::Affine>::Identity();

	// Initialize the end_effector_state by the Goal State co-ordinates
	end_effector_state *= Eigen::Translation<double,3>(goalPose[0], goalPose[1], goalPose[2]);

	// Define a Geomatric pose and initialize it with zeros
	geometry_msgs::Pose pose;
	
	pose.position.x = 0.0;
	pose.position.y = 0.0;
	pose.position.z = 0.0;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.0;
	pose.orientation.w = 0.0;

	geometry_msgs::Pose &pose_1 = pose;

	// Convert the end_effector_state from an Eigen Pose to a Geomatric message
	tf::poseEigenToMsg(end_effector_state, pose_1);

	// Print end-effector pose. Remember that this is in the model frame
	ROS_INFO_STREAM("\nTranslation Matrix: " << end_effector_state.translation());
	ROS_INFO_STREAM("\nRotation Matrix: \n" << end_effector_state.rotation() << "\n");

	ROS_INFO_STREAM("Translation in X: " << pose.position.x);
	ROS_INFO_STREAM("Translation in Y: " << pose.position.y);
	ROS_INFO_STREAM("Translation in Z: " << pose.position.z);

	ROS_INFO_STREAM("Rotation in W: " << pose.orientation.w);
	ROS_INFO_STREAM("Rotation in X: " << pose.orientation.x);
	ROS_INFO_STREAM("Rotation in Y: " << pose.orientation.y);
	ROS_INFO_STREAM("Rotation in Z: " << pose.orientation.z);

	// Look for an IK Solution in our Joint Model Group for the end_effector_state Goal State
	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

	// In case of failure, keep looking for a solution for a specific number of trials
	if(found_ik == false && count<max_trial)
	{
		count++;
		ROS_INFO_STREAM("----------------------------------\n\nLooking for an IK Solution..... [count -> " << count << "]\n");
		//found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);
		
		checkIK(goalPose, node_handle, count);
	}

	// Now, we can print out the IK solution (if found):
	if (found_ik)
	{
		ROS_INFO("Congratulations!! We have found an IK Solution for the Goal State..");
		ROS_INFO("Goal State Joint Values are as follows: ");

		// Copy the Joint Group positions for the Goal State
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		for(std::size_t i=0; i < joint_names.size(); ++i)
		{
			ROS_INFO("Joint Name %s: 	Value: %f", joint_names[i].c_str(), joint_values[i]);
		}
		
		// We can also get the Jacobian from the :moveit_core:`RobotState`.
		Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
		Eigen::MatrixXd jacobian;

		kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),reference_point_position,jacobian);
		ROS_INFO_STREAM("\nJacobian Matrix: \n" << jacobian);

		// Define a point for representing the Goal State for the robot
		trajectory_msgs::JointTrajectoryPoint point;

		// Initialize a time_from_start for moveit to kick-start Motion Planning
		point.time_from_start = ros::Duration(1.0);

		// Fill up the Joint Group positions for this point[Goal State]
		for(std::size_t i=0; i < joint_names.size()-2; ++i)
			{
				point.positions.push_back(joint_values[i]);
				
				/* Optionally set Velocities and Accelarations for the Motion Planing
				point.velocities.push_back(1);
				point.accelerations.push_back(1); */
			}

			// Push the Goal State in our FollowJointTrajectoryActionGoal Variable		
			action.goal.trajectory.points.push_back(point);

		sleep_time.sleep();
	}

	if(found_ik == false && count>=max_trial)
	{
		ROS_INFO("Sorry, Did not find IK solution..");
	}

	if(found_ik == true && count>=max_trial)
		return true;

	else
		return false;
}

// Check to see if you will need to start the spinner here or in the Trial.. If in the Trial, then do you need to define the subscribers/publishers there and pass it here?
bool pickANDplace::execute(std::vector<double> goalPose, ros::NodeHandle node_handle)
{
	ros::AsyncSpinner spinner(1);
	spinner.start();

	joints = node_handle.subscribe(jointState_topic, 1000, &pickANDplace::getJointStates, this);
	
	// Create a coutner to check for an IK in case of failure
	int count = 0;

	bool IK = false;

	// Check if an IK Solution exists for the Goal State	
	IK = checkIK(goalPose, node_handle, count);

	return IK;
}

bool pickANDplace::pick(ros::NodeHandle node_handle)
{
	ros::WallDuration sleep_time(0.0);
	sleep_time.sleep();

	joints = node_handle.subscribe(jointState_topic, 1000, &pickANDplace::getJointStates, this);

	// Define 'arm' and 'gripper' control ROS publishers
	arm_publisher = node_handle.advertise<control_msgs::FollowJointTrajectoryActionGoal>(armPublisher_topic, 1, true);

	// Publish Joint Commands on the arm_publisher from the planned trajectory
	
	arm_publisher.publish(action);
	sleep_time.sleep();

	return true;
}

bool pickANDplace::place(ros::NodeHandle node_handle)
{
	ros::WallDuration sleep_time(0.0);
	sleep_time.sleep();

	joints = node_handle.subscribe(jointState_topic, 1000, &pickANDplace::getJointStates, this);

	// Define 'arm' and 'gripper' control ROS publishers
	arm_publisher = node_handle.advertise<std_msgs::Float64>(gripperPublisher_topic, 1000);

	// Publish Joint Commands on the arm_publisher from the planned trajectory
	arm_publisher.publish(action);
	sleep_time.sleep();

	return true;
}

bool pickANDplace::grasp(ros::NodeHandle node_handle, double msg)
{
	ros::WallDuration sleep_time(0.0);
	sleep_time.sleep();

	joints = node_handle.subscribe(jointState_topic, 1000, &pickANDplace::getJointStates, this);

	// Define 'gripper' control ROS publishers
	gripper_publisher = node_handle.advertise<std_msgs::Float64>(gripperPublisher_topic, 1, true);

	// Publish Joint Commands on the gripper_publisher from the planned trajectory
	sleep_time.sleep();
	std_msgs::Float64 grip;
	grip.data = msg;
	gripper_publisher.publish(grip);

	return true;
}

void pickANDplace::getJointStates(const sensor_msgs::JointStatePtr& msg)
{
	// Empty the Map container
	curr_joint_values.clear();

	for(int jointList=0; jointList < msg->name.size(); jointList++)
	{
		curr_joint_values[msg->name[jointList]] = msg->position[jointList];
		// ROS_INFO("Receiving [Joint] %s -> %f ", msg->name[jointList].c_str(), msg->position[jointList]);
	}

	return;
}

string pickANDplace::getReferenceFrame()
{
	return referenceFrame;
}

std::vector<double> pickANDplace::getRestPosition()
{
	return rest_coordinates;
}

std::vector<double> pickANDplace::getHandlePosition()
{
	return handle_coordinates;
}

std::vector<double> pickANDplace::getPickUpLoc()
{
	return pickup_coordinates;
}

std::vector<double> pickANDplace::getPlaceLoc()
{
	return place_coordinates;
}

void pickANDplace::setObjectID(string id)
{
	objectID = id;
}

void pickANDplace::setObjectThickness(double thickness)
{
	objectThickness = thickness;
}

void pickANDplace::setSimulation(bool sim_status)
{
	isSimulation = sim_status;
}

void pickANDplace::setGripperOpenLimit(double gripperOpen_Limit)
{
	gripper_open = gripperOpen_Limit;
}

void pickANDplace::setGripperGraspLimit(double gripperGrasp_Limit)
{
	gripper_grasp = gripperGrasp_Limit;
}

void pickANDplace::setGripperCloseLimit(double gripperClose_Limit)
{
	gripper_close = gripperClose_Limit;
}

void pickANDplace::setReferenceFrame(string ref)
{
	referenceFrame = ref;
}

void pickANDplace::setRestPosition(std::vector<double> rest)
{
	rest_coordinates = rest;
}

void pickANDplace::setHandlePosition(std::vector<double> handle)
{
	handle_coordinates = handle;
}

void pickANDplace::setPickUpLoc(std::vector<double> pickup)
{
	pickup_coordinates = pickup;
}

void pickANDplace::setPlaceLoc(std::vector<double> place)
{
	place_coordinates = place;
}