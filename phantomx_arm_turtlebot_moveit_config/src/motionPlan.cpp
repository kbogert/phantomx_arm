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

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "move_group_tutorial");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle("motion_planner_api");

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	std::string planner_plugin_name;

	if (!node_handle.getParam("/move_group/planning_plugin", planner_plugin_name))
	ROS_FATAL_STREAM("Could not find planner plugin name");
	try
	{
		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
	}
	try
	{
		planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
		if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
		ROS_FATAL_STREAM("Could not initialize planner instance");
		ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
	}
	catch(pluginlib::PluginlibException& ex)
	{
		const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
		std::stringstream ss;
		for (std::size_t i = 0 ; i < classes.size() ; ++i)
		ss << classes[i] << " ";
		ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl << "Available plugins: " << ss.str());
	}

	ros::WallDuration sleep_time(15.0);
	sleep_time.sleep(); 

	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;

	geometry_msgs::PoseStamped pose;
	
	pose.header.frame_id = "odom";
	pose.pose.position.x = -0.0119214;
	pose.pose.position.y = 0.0972478;
	pose.pose.position.z = 0.636616;
	pose.pose.orientation.x = 0.273055;
	pose.pose.orientation.y = -0.891262;
	pose.pose.orientation.z = -0.346185;
	pose.pose.orientation.w = 0.106058;

	group.setPoseTarget(target_pose1);

	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);

	if (1)
	{
		ROS_INFO("Visualizing plan 1 (again)");
		display_trajectory.trajectory_start = my_plan.start_state_;
		display_trajectory.trajectory.push_back(my_plan.trajectory_);
		display_publisher.publish(display_trajectory);
		/* Sleep to give Rviz time to visualize the plan. */
		sleep(5.0);
	}

	std::vector<double> group_variable_values;
	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

	group_variable_values[4] = -1.0;
	group.setJointValueTarget(group_variable_values);
	success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");

	sleep(5.0);

	

	return 0;
}





// ROS Includes
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt Includes
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "move_group_tutorial");
	ros::NodeHandle node_handle("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	sleep(20.0);

	moveit::planning_interface::MoveGroup group("arm");

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 

	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

	geometry_msgs::PoseStamped target_pose1;
	
	target_pose1.header.frame_id = "odom";
	target_pose1.pose.position.x = 0.128518;
	target_pose1.pose.position.y = -0.132719;
	target_pose1.pose.position.z = 0.321876;
	target_pose1.pose.orientation.w = 0.0125898;
	target_pose1.pose.orientation.x = 0.184773;
	target_pose1.pose.orientation.y = -0.980428;
	target_pose1.pose.orientation.z = -0.0667877;
		
	group.setPoseTarget(target_pose1);

	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED"); 

	sleep(5.0);

	if (1)
	{
		ROS_INFO("Visualizing plan 1 (again)");
		display_trajectory.trajectory_start = my_plan.start_state_;
		display_trajectory.trajectory.push_back(my_plan.trajectory_);
		display_publisher.publish(display_trajectory);
		/* Sleep to give Rviz time to visualize the plan. */
		sleep(5.0);
	}

	ros::shutdown();
	return 0;

	return 0;
}