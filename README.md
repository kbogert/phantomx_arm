IMPORTANT: This Library was developed as part of a Masters Thesis on Inverse Reinforcement Learning for Multi-Robot coordination in an Ad-Hoc setting. The Machine Learning part of the code, as well the experimental setup for this research is not publicly available.

phantomx_arm turtlebot Library
(Launch files marked by * (and the subsequent source files) are not uploaded due to privacy issues)

ROS GAZEBO SIMULATOR EXPERIMENT LAUNCH: 
In order to launch the experiments in Gazebo, go to the terminal and start the following launch files (in separate terminals)..
NOTE: To understand code structure and program flow, FOLLOW THE LAUNCH FILES

Single Robot Experiment Launch:
 roslaunch phantomx_arm_turtlebot_moveit_config gazeboPhantomX.launch 
 roslaunch phantomx_arm_turtlebot_moveit_config gazeboArmController.launch
 roslaunch phantomx_arm_turtlebot_moveit_config amcl_robots_robot1.launch/roslaunch phantomx_arm_turtlebot_moveit_config amcl_robots_robot2.launch
 	(Robot 1 is the Expert and Robot 2 is the Learner)
 *roslaunch phantomx_arm_turtlebot_moveit_config gazebo_expert_mdp.launch/roslaunch phantomx_arm_turtlebot_moveit_config gazebo_learner_mdp.launch
 
*Multi-Robot Experiment Launch:
 roslaunch phantomx_arm_turtlebot_moveit_config gazeboMultiPhantomX.launch 
 roslaunch phantomx_arm_turtlebot_moveit_config amcl_robots_robot1.launch
 roslaunch phantomx_arm_turtlebot_moveit_config amcl_robots_robot2.launch
 	(Robot 1 is the Expert and Robot 2 is the Learner)
 *roslaunch phantomx_arm_turtlebot_moveit_config gazebo_expert_mdp.launch
 *roslaunch phantomx_arm_turtlebot_moveit_config gazebo_two_robot_learner_mdp.launch

Testing IK and Motion Planning (Single Robot):
 roslaunch phantomx_arm_turtlebot_moveit_config gazeboPhantomX.launch
 roslaunch phantomx_arm_turtlebot_moveit_config gazeboArmController.launch
 roslaunch phantomx_arm_turtlebot_moveit_config kinematicSolver.launch/roslaunch phantomx_arm_turtlebot_moveit_config pickANDplace.launch

IMPORTANT NOTES:
 Make sure the namespaces in the Launch files match the ones in the respective .cpp files
 Any timeout errors resulting from tf transforms can be traced back by generating the tf tree (rosrun tf view_frames/rosrun tf tf_monitor)
 
PHYSICAL ROBOT EXPERIMENT LAUNCH: 
In order to launch the experiments on the physical robot, go to the terminal and start the following launch files (in separate terminals)..
NOTE: To understand code structure and program flow, FOLLOW THE LAUNCH FILES

Physical Robot Experiment Launch:
 roslaunch phantomx_arm_turtlebot_moveit_config arm.launch
 roslaunch phantomx_arm_turtlebot_moveit_config test.launch

Testing IK and Motion Planning (Single Robot):
 roslaunch phantomx_arm_turtlebot_moveit_config kinematicSolver.launch/roslaunch phantomx_arm_turtlebot_moveit_config pickANDplace.launch

IMPORTANT NOTES:
 Make sure the namespaces in the Launch files match the ones in the respective .cpp files
 Any timeout errors resulting from tf transforms can be traced back by generating the tf tree (rosrun tf view_frames/rosrun tf tf_monitor)
