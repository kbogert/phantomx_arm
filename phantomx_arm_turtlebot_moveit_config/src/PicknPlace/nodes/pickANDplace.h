#ifndef PICKANDPLACE_H
#define PICKANDPLACE_H

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

namespace picknplace
{
	using namespace std;

	class pickANDplace
	{
		public:
			
			// Class variables
			ros::Subscriber joints;
			ros::Publisher arm_publisher;
			ros::Publisher gripper_publisher;

			string objectID;
			double objectThickness;

			double gripper_open;
			double gripper_grasp;
			double gripper_close;
			
			bool flag;
			bool execution;
			bool isSimulation;

			std::vector<double> rest_coordinates;
			std::vector<double> handle_coordinates;
			std::vector<double> pickup_coordinates;
			std::vector<double> place_coordinates;

			std::map <string, double> curr_joint_values;

			string referenceFrame;
			string armPublisher_topic;
			string gripperPublisher_topic;
			string jointState_topic;
			
			int max_trial;

			control_msgs::FollowJointTrajectoryActionGoal action;

			// Constructor/Destructor
			pickANDplace();
			~pickANDplace();

			// Member Functions
			void setObjectID(string id);
			void setReferenceFrame(string ref);
			void setObjectThickness(double thickness);
			void setSimulation(bool sim_status);

			void setGripperOpenLimit(double gripperOpen_Limit);
			void setGripperGraspLimit(double gripperGrasp_Limit);
			void setGripperCloseLimit(double gripperClose_Limit);

			void setRestPosition(std::vector<double> rest);
			void setHandlePosition(std::vector<double> handle);
			void setPickUpLoc(std::vector<double> pickup);
			void setPlaceLoc(std::vector<double> place);

			void getJointStates(const sensor_msgs::JointStatePtr& msg);

			string getReferenceFrame();

			std::vector<double> getRestPosition();
			std::vector<double> getHandlePosition();
			std::vector<double> getPickUpLoc();
			std::vector<double> getPlaceLoc();

			bool checkIK(std::vector<double> goalPose, ros::NodeHandle node_handle, int count);
			bool execute(std::vector<double> goalPose, ros::NodeHandle node_handle); // Go to Rest --> Open --> Pick Up --> Grasp --> Go to Handle --> Place --> Open --> Go to Rest --> Close

			bool pick(ros::NodeHandle node_handle);
			bool place(ros::NodeHandle node_handle);
			bool grasp(ros::NodeHandle node_handle, double msg);
			bool open(ros::NodeHandle node_handle);
			bool close(ros::NodeHandle node_handle);

	};
}

#endif