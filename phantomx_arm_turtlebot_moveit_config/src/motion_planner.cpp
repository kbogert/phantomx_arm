// ROS Includes
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt Includes
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <moveit_msgs/PlanningScene.h>

using namespace std;

int main(int argc, char **argv)
{
	ros::init (argc, argv, "move_group_tutorial");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle("motion_planner_api");

	ros::ServiceClient service_client = node_handle.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
	
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

	planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

	ros::WallDuration sleep_time(8.0);
	sleep_time.sleep();
	
	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;

	geometry_msgs::PoseStamped pose;
	
	pose.header.frame_id = "odom";
	
	pose.pose.position.x = -0.176234;
	pose.pose.position.y =  0.00074865;
	pose.pose.position.z = 0.595991;
	pose.pose.orientation.w = 0.554316;
	pose.pose.orientation.x = -0.436169;
	pose.pose.orientation.y = -0.436175;
	pose.pose.orientation.z = 0.55432;

	std::vector<double> tolerance_pose(3, 0.01);
	std::vector<double> tolerance_angle(3, 0.01);

	req.group_name = "arm";
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("link_5", pose, tolerance_pose, tolerance_angle);
	req.goal_constraints.push_back(pose_goal);

	planning_pipeline->generatePlan(planning_scene, req, res);
	
	if(res.error_code_.val != res.error_code_.SUCCESS)
	{
	  ROS_ERROR("Could not compute plan successfully");
	  return 0;
	}

	/* Visualize the trajectory */

	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	ROS_INFO("Visualizing the trajectory");
	moveit_msgs::MotionPlanResponse response;
	res.getMessage(response);

	display_trajectory.trajectory_start = response.trajectory_start;
	display_trajectory.trajectory.push_back(response.trajectory);
	display_publisher.publish(display_trajectory);

/*	----------------------------------->> THIS SHOULD WORK?  <<-----------------------------------------------*/
	trajectory_execution_manager::TrajectoryExecutionManager execute_Trajectory(robot_model, true);
	moveit_msgs::RobotTrajectory &robot_Trajectory = display_trajectory.trajectory[0];

	std::string new_string;
	int len = 10;
	char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";

    for (int i = 0; i < len; ++i) {
        new_string[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
    }

    new_string[len] = 0;

	execute_Trajectory.enableExecutionDurationMonitoring(false);
	execute_Trajectory.pushAndExecute(robot_Trajectory, "arm_controller");

	sleep_time.sleep();

	/* Execute the Trajectory on the Arm */

	//ROS_INFO("Executing the Trajectory..");

	ros::Publisher arm_publisher = node_handle.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal", 1, true);
	control_msgs::FollowJointTrajectoryActionGoal action;
	control_msgs::FollowJointTrajectoryGoal goal;

	ros::Time begin = ros::Time::now();
	
	action.header.stamp = begin + ros::Duration(1.0);

	action.goal_id.stamp = begin + ros::Duration(1.0);
	action.goal_id.id = new_string;

	action.goal.trajectory = (response.trajectory.joint_trajectory);

	int check = action.goal.trajectory.points.size();
	ROS_INFO("VECTOR SIZE: ");


	arm_publisher.publish(action);

	// ------------------> USING ROS Service

	/*ros::ServiceClient client = node_handle.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");
	moveit_msgs::ExecuteKnownTrajectory armTrajectory;

	armTrajectory.request.trajectory = response.trajectory;

	ros::service::waitForService("execute_kinematic_path");

	if (client.call(armTrajectory))
    {
   		ROS_INFO("EXECUTION SUCCESSFUL!!");
    }
    else
    {
  		ROS_ERROR("Failed to call service execute_kinematic_path");
   	}*/


   sleep_time.sleep();

	return 0;
}