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

// In order to keep a good structure and not mess with the Boost bindings we create a separate class for jointStateSubscriber
class jointStateSubscriber
{
	public:
			
	// Class variables
	string jointState_topic;
	ros::Subscriber joints;

	// Constructor
	jointStateSubscriber(ros::NodeHandle node_handle);

	//Member Functions
	void getJointStates(const sensor_msgs::JointStatePtr& msg);

};
