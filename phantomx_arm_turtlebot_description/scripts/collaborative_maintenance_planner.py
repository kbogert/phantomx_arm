#! /usr/bin/env python

# Import move base msgs for action and goal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import *
from std_msgs.msg import *
from utils.navigation import *
import rospy
import initial_pose
import move_to_goal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from utils.transform_utils import *
from math import sqrt, pow, radians
import tf

class maintenance_planner():
    	def run(self):	
		# Stow robotic arm
		# Create a joint state publisher for each turtlebot
		self.arm_1_pub = rospy.Publisher('turtlebot_1/arm_controller/command', JointTrajectory)
		self.arm_2_pub = rospy.Publisher('turtlebot_2/arm_controller/command', JointTrajectory)
		rospy.sleep(1)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for grasp position
		rospy.loginfo("Position arm in stowed position")
		rospy.sleep(3)
		msg = JointTrajectory()
		msg.header.seq = 1
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = '0'
		msg.joint_names.append('joint_1')
		msg.joint_names.append('joint_2')
		msg.joint_names.append('joint_3')
		msg.joint_names.append('joint_4')
		msg.joint_names.append('joint_5')
		# Define joint trajectory points
		point = JointTrajectoryPoint()
		point.positions  = [0.0, -1.3, 1.7, 1.2, 0.1]
		point.velocities = [50.0, 50.0, 50.0, 50.0, 10.0]
		point.time_from_start = rospy.Duration(3.0)
		msg.points.append(point)
            	# publish joint stage message
            	self.arm_1_pub.publish(msg)
		self.arm_2_pub.publish(msg)
		rospy.sleep(3)

		# Run each stage of maintenance
		#self.stage_4()
		#self.stage_3()
		#self.stage_5()

		self.stage_4_part_1()
		self.stage_3_part_1()
		self.stage_5_part_1()
		
		self.stage_4_part_2()
		self.stage_3_part_2()
		self.stage_5_part_2()

		self.stage_6()
		self.stage_7()
		self.stage_8()
		self.stage_9()

	def stage_1(self):

		# Move to initial position using move base goal
		rospy.loginfo("Begin stage 1")
		# Move turtlebot 1 to starting position
		# Intialize the waypoint goal
            	goal = MoveBaseGoal()
            	# Use the map frame to define goal poses
            	goal.target_pose.header.frame_id = 'map'
            	# Set pose
        	goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = 0.840401411057
        	goal.target_pose.pose.position.y = 0.674615263939
        	goal.target_pose.pose.orientation = Quaternion(0.0,0.0,-0.631627191685, 0.77527226877)
		# Move to goal
		move_to_goal.move_to_goal(goal)
		# Scan for ar marker and get position + offset to reposition turtlebot
		# Set pose THIS IS WRONG, based upon AR marker which has frame id rgb camera
        	goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = 0.0927959606051
        	goal.target_pose.pose.position.y = -0.157733559608
        	goal.target_pose.pose.orientation = Quaternion(0.0,0.0,-0.0746215653308, 0.0235089268487)
		# Move to goal
		move_to_goal.move_to_goal(goal, 'turtlebot_1')

		# Move turtlebot 2 to starting position
		# Intialize the waypoint goal
            	goal = MoveBaseGoal()
            	# Use the map frame to define goal poses
            	goal.target_pose.header.frame_id = 'map'
            	# Set pose
        	goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = 0.840401411057
        	goal.target_pose.pose.position.y = 0.674615263939
        	goal.target_pose.pose.orientation = Quaternion(0.0,0.0,-0.631627191685, 0.77527226877)
		# Move to goal
		move_to_goal.move_to_goal(goal)
		# Scan for ar marker and get position + offset to reposition turtlebot
		# Set pose THIS IS WRONG, based upon AR marker which has frame id rgb camera
        	goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = 0.0927959606051
        	goal.target_pose.pose.position.y = -0.157733559608
        	goal.target_pose.pose.orientation = Quaternion(0.0,0.0,-0.0746215653308, 0.0235089268487)
		# Move to goal
		move_to_goal.move_to_goal(goal, 'turtlebot_2')

	def stage_2(self):

		# Move to initial position using move base goal
		rospy.loginfo("Begin stage 2")

	def stage_3(self):

		# Move arm to grasping position
		rospy.loginfo("Begin stage 3")
		# Create a joint state publisher
		self.arm_1_pub = rospy.Publisher('turtlebot_1/arm_controller/command', JointTrajectory)
		self.arm_2_pub = rospy.Publisher('turtlebot_2/arm_controller/command', JointTrajectory)
		rospy.sleep(1)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for grasp position
		rospy.loginfo("Position arm in grasp position")
		msg = JointTrajectory()
		msg.header.seq = 1
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = '0'
		msg.joint_names.append('joint_1')
		msg.joint_names.append('joint_2')
		msg.joint_names.append('joint_3')
		msg.joint_names.append('joint_4')
		msg.joint_names.append('joint_5')
		# Define joint trajectory points
		point = JointTrajectoryPoint()
		point.positions  = [0.0, 0.0, 0.0, 1.6, 0.1]
		point.velocities = [20.0, 20.0, 20.0, 10.0, 10.0]
		point.time_from_start = rospy.Duration(3.0)
		msg.points.append(point)
            	# publish joint stage message
            	self.arm_1_pub.publish(msg)	
		self.arm_2_pub.publish(msg)	
		rospy.sleep(3)

	def stage_3_part_1(self):

		# Move arm to grasping position
		rospy.loginfo("Begin stage 3 part 1")
		# Create a joint state publisher
		self.arm_1_pub = rospy.Publisher('turtlebot_1/arm_controller/command', JointTrajectory)
		rospy.sleep(1)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for grasp position
		rospy.loginfo("Position arm in grasp position")
		msg = JointTrajectory()
		msg.header.seq = 1
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = '0'
		msg.joint_names.append('joint_1')
		msg.joint_names.append('joint_2')
		msg.joint_names.append('joint_3')
		msg.joint_names.append('joint_4')
		msg.joint_names.append('joint_5')
		# Define joint trajectory points
		point = JointTrajectoryPoint()
		point.positions  = [0.0, 0.0, 0.0, 1.6, 0.1]
		point.velocities = [20.0, 20.0, 20.0, 10.0, 10.0]
		point.time_from_start = rospy.Duration(3.0)
		msg.points.append(point)
            	# publish joint stage message
            	self.arm_1_pub.publish(msg)	
		rospy.sleep(3)

	def stage_3_part_2(self):

		# Move arm to grasping position
		rospy.loginfo("Begin stage 3 part 2")
		# Create a joint state publisher
		self.arm_2_pub = rospy.Publisher('turtlebot_2/arm_controller/command', JointTrajectory)
		rospy.sleep(1)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for grasp position
		rospy.loginfo("Position arm in grasp position")
		msg = JointTrajectory()
		msg.header.seq = 1
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = '0'
		msg.joint_names.append('joint_1')
		msg.joint_names.append('joint_2')
		msg.joint_names.append('joint_3')
		msg.joint_names.append('joint_4')
		msg.joint_names.append('joint_5')
		# Define joint trajectory points
		point = JointTrajectoryPoint()
		point.positions  = [0.0, 0.0, 0.0, 1.6, 0.1]
		point.velocities = [20.0, 20.0, 20.0, 10.0, 10.0]
		point.time_from_start = rospy.Duration(3.0)
		msg.points.append(point)
            	# publish joint stage message
		self.arm_2_pub.publish(msg)	
		rospy.sleep(3)

	def stage_4(self):

		# Move turtlebot to grasping position
		rospy.loginfo("Begin stage 4")
		# Publisher to control the robot's speed
     		self.cmd_vel_1 = rospy.Publisher('turtlebot_1/cmd_vel_mux/input/navi', Twist)
		self.cmd_vel_2 = rospy.Publisher('turtlebot_2/cmd_vel_mux/input/navi', Twist)
		# How fast will we update the robot's movement? in Hz
        	rate = 20
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Set the forward linear speed to 0.1 meters per second
        	linear_speed = 0.1
        	# Set the travel distance in meters
        	goal_distance = 0.65
		#################################################################################
		# Initialize the tf listener
        	self.tf_listener = tf.TransformListener()
        	# Give tf some time to fill its buffer
        	rospy.sleep(2)
        	# Set the odom frame
        	self.odom_frame = 'turtlebot_1/odom'
		self.base_frame = 'turtlebot_1/base_footprint'
		# Initialize the position variable as a Point type
        	position_1 = Point()
		# Initialize the movement command
            	move_cmd_1 = Twist()
            	# Set the movement command to forward motion
            	move_cmd_1.linear.x = linear_speed
		rospy.loginfo("Before" + " " + self.odom_frame + " " + self.base_frame)
            	# Get the starting position values
            	(position_1, rotation) = get_odom(self)      
		rospy.loginfo("after")
            	x_start_1 = position_1.x
            	y_start_1 = position_1.y
		###############################################################################
        	# Set the odom frame
        	self.odom_frame = 'turtlebot_2/odom'
		self.base_frame = 'turtlebot_2/base_footprint'
		# Initialize the position variable as a Point type
        	position_2 = Point()
		# Initialize the movement command
            	move_cmd_2 = Twist()
            	# Set the movement command to forward motion
            	move_cmd_2.linear.x = linear_speed
            	# Get the starting position values
            	(position_2, rotation) = get_odom(self)      
            	x_start_2 = position_2.x
            	y_start_2 = position_2.y

            	# Keep track of the distance traveled
            	distance_1 = 0
		distance_2 = 0
		rospy.loginfo("Begin moving turtlebot forward")
		# Enter the loop to move turtlebot
            	while distance_1 < goal_distance or distance_2 < goal_distance and not rospy.is_shutdown():
                	# Publish the Twist message and sleep 1 cycle
                	self.cmd_vel_1.publish(move_cmd_1)
			self.cmd_vel_2.publish(move_cmd_2)
                	# sleep
                	r.sleep()
                	# Get the current position
			# Set the odom frame
        		self.odom_frame = 'turtlebot_1/odom'
			self.base_frame = 'turtlebot_1/base_footprint'
                	(position_1, rotation_1) = get_odom(self)
			# Set the odom frame
        		self.odom_frame = 'turtlebot_2/odom'
			self.base_frame = 'turtlebot_2/base_footprint'
			(position_2, rotation_2) = get_odom(self)
                	# Compute the Euclidean distance from the start
                	distance_1 = sqrt(pow((position_1.x - x_start_1), 2) +
                                	pow((position_1.y - y_start_1), 2))
			distance_2 = sqrt(pow((position_2.x - x_start_2), 2) +
                                	pow((position_2.y - y_start_2), 2))
			# Check whether we need to stop one of the turtlebots early
			if distance_1 >= goal_distance:
				move_cmd_1 = Twist()
            			self.cmd_vel_1.publish(move_cmd_1)
			if distance_2 >= goal_distance:
				move_cmd_2 = Twist()
            			self.cmd_vel_2.publish(move_cmd_2)

		# Stop the robot before reversing
		rospy.loginfo("Stop turtlebot")            	
		move_cmd_1 = Twist()
            	self.cmd_vel_1.publish(move_cmd_1)
		move_cmd_2 = Twist()
            	self.cmd_vel_2.publish(move_cmd_2)
            	rospy.sleep(1)

	def stage_4_part_1(self):

		# Move turtlebot to grasping position
		rospy.loginfo("Begin stage 4 part 1")
		# Publisher to control the robot's speed
     		self.cmd_vel_1 = rospy.Publisher('turtlebot_1/cmd_vel_mux/input/navi', Twist)
		# How fast will we update the robot's movement? in Hz
        	rate = 20
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Set the forward linear speed to 0.1 meters per second
        	linear_speed = 0.1
        	# Set the travel distance in meters
        	goal_distance = 0.65
		#################################################################################
		# Initialize the tf listener
        	self.tf_listener = tf.TransformListener()
        	# Give tf some time to fill its buffer
        	rospy.sleep(2)
        	# Set the odom frame
        	self.odom_frame = 'turtlebot_1/odom'
		self.base_frame = 'turtlebot_1/base_footprint'
		# Initialize the position variable as a Point type
        	position_1 = Point()
		# Initialize the movement command
            	move_cmd_1 = Twist()
            	# Set the movement command to forward motion
            	move_cmd_1.linear.x = linear_speed
		rospy.loginfo("Before" + " " + self.odom_frame + " " + self.base_frame)
            	# Get the starting position values
            	(position_1, rotation) = get_odom(self)      
		rospy.loginfo("after")
            	x_start_1 = position_1.x
            	y_start_1 = position_1.y
            	# Keep track of the distance traveled
            	distance_1 = 0
		rospy.loginfo("Begin moving turtlebots forward")
		# Enter the loop to move turtlebot
            	while distance_1 < goal_distance and not rospy.is_shutdown():
                	# Publish the Twist message and sleep 1 cycle
                	self.cmd_vel_1.publish(move_cmd_1)
                	# sleep
                	r.sleep()
                	# Get the current position
			# Set the odom frame
        		self.odom_frame = 'turtlebot_1/odom'
			self.base_frame = 'turtlebot_1/base_footprint'
                	(position_1, rotation_1) = get_odom(self)
                	# Compute the Euclidean distance from the start
                	distance_1 = sqrt(pow((position_1.x - x_start_1), 2) +
                                	pow((position_1.y - y_start_1), 2))
			# Check whether we need to stop one of the turtlebots early
			if distance_1 >= goal_distance:
				move_cmd_1 = Twist()
            			self.cmd_vel_1.publish(move_cmd_1)

		# Stop the robot before reversing
		rospy.loginfo("Stop turtlebots")            	
		move_cmd_1 = Twist()
            	self.cmd_vel_1.publish(move_cmd_1)
            	rospy.sleep(1)

	def stage_4_part_2(self):

		# Move turtlebot to grasping position
		rospy.loginfo("Begin stage 4 part 2")
		# Publisher to control the robot's speed
		self.cmd_vel_2 = rospy.Publisher('turtlebot_2/cmd_vel_mux/input/navi', Twist)
		# How fast will we update the robot's movement? in Hz
        	rate = 20
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Set the forward linear speed to 0.1 meters per second
        	linear_speed = 0.1
        	# Set the travel distance in meters
        	goal_distance = 0.65
		# Initialize the tf listener
        	self.tf_listener = tf.TransformListener()
        	# Give tf some time to fill its buffer
        	rospy.sleep(2)
		###############################################################################
        	# Set the odom frame
        	self.odom_frame = 'turtlebot_2/odom'
		self.base_frame = 'turtlebot_2/base_footprint'
		# Initialize the position variable as a Point type
        	position_2 = Point()
		# Initialize the movement command
            	move_cmd_2 = Twist()
            	# Set the movement command to forward motion
            	move_cmd_2.linear.x = linear_speed
            	# Get the starting position values
            	(position_2, rotation) = get_odom(self)      
            	x_start_2 = position_2.x
            	y_start_2 = position_2.y

            	# Keep track of the distance traveled
		distance_2 = 0
		rospy.loginfo("Begin moving turtlebot forward")
		# Enter the loop to move turtlebot
            	while distance_2 < goal_distance and not rospy.is_shutdown():
                	# Publish the Twist message and sleep 1 cycle
			self.cmd_vel_2.publish(move_cmd_2)
                	# sleep
                	r.sleep()
                	# Get the current position
			# Set the odom frame
        		self.odom_frame = 'turtlebot_2/odom'
			self.base_frame = 'turtlebot_2/base_footprint'
			(position_2, rotation_2) = get_odom(self)
                	# Compute the Euclidean distance from the start
			distance_2 = sqrt(pow((position_2.x - x_start_2), 2) +
                                	pow((position_2.y - y_start_2), 2))
			# Check whether we need to stop one of the turtlebots early
			if distance_2 >= goal_distance:
				move_cmd_2 = Twist()
            			self.cmd_vel_2.publish(move_cmd_2)

		# Stop the robot before reversing
		rospy.loginfo("Stop turtlebot")            	
		move_cmd_2 = Twist()
            	self.cmd_vel_2.publish(move_cmd_2)
            	rospy.sleep(1)

	def stage_5(self):

		# Close gripper
		rospy.loginfo("Begin stage 5")
		# Create a joint state publisher
		self.joint5_pub_1 = rospy.Publisher('turtlebot_1/joint_5/command', Float64)
		self.joint5_pub_2 = rospy.Publisher('turtlebot_2/joint_5/command', Float64)
		rospy.sleep(2)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for joint 5, gripper to close
		rospy.loginfo("Position joint 5")
		msg = -1.8
            	# publish joint stage message
            	self.joint5_pub_1.publish(msg)
		self.joint5_pub_2.publish(msg)
		rospy.sleep(1)

	def stage_5_part_1(self):

		# Close gripper
		rospy.loginfo("Begin stage 5 part 1")
		# Create a joint state publisher
		self.joint5_pub_1 = rospy.Publisher('turtlebot_1/joint_5/command', Float64)
		rospy.sleep(2)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for joint 5, gripper to close
		rospy.loginfo("Position joint 5")
		msg = -1.8
            	# publish joint stage message
            	self.joint5_pub_1.publish(msg)
		rospy.sleep(1)

	def stage_5_part_2(self):

		# Close gripper
		rospy.loginfo("Begin stage 5 part 2")
		# Create a joint state publisher
		self.joint5_pub_2 = rospy.Publisher('turtlebot_2/joint_5/command', Float64)
		rospy.sleep(2)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for joint 5, gripper to close
		rospy.loginfo("Position joint 5")
		msg = -1.8
            	# publish joint stage message
		self.joint5_pub_2.publish(msg)
		rospy.sleep(1)

	def stage_6(self):

		# Reverse turtlebot a specfic distance and lower arm at a specific velocity to target position
		rospy.loginfo("Begin stage 6")
		# Publisher to control the robot's speed
     		self.cmd_vel_1 = rospy.Publisher('turtlebot_1/cmd_vel_mux/input/navi', Twist)
		self.cmd_vel_2 = rospy.Publisher('turtlebot_2/cmd_vel_mux/input/navi', Twist)
		# Create a joint state publisher
		self.arm_pub_1 = rospy.Publisher('turtlebot_1/arm_controller/command', JointTrajectory)
		self.arm_pub_2 = rospy.Publisher('turtlebot_2/arm_controller/command', JointTrajectory)
		# Initialize the tf listener
        	self.tf_listener = tf.TransformListener()
        	# Give tf some time to fill its buffer
        	rospy.sleep(1)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for support position
		rospy.loginfo("Position arm in support position")
		msg = JointTrajectory()
		msg.header.seq = 1
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = '0'
		msg.joint_names.append('joint_1')
		msg.joint_names.append('joint_2')
		msg.joint_names.append('joint_3')
		msg.joint_names.append('joint_4')
		msg.joint_names.append('joint_5')
		# Define joint trajectory points
		point = JointTrajectoryPoint()
		point.positions  = [0.0, 1.0, -1.0, 1.0, -1.8]
		point.velocities = [2.0, 2.0, 20.0, 10.0, 1.0]
		point.time_from_start = rospy.Duration(4.0)
		msg.points.append(point)
		# Define joint trajectory points
		point = JointTrajectoryPoint()
		point.positions  = [0.0, 1.5, 0.0, 0.0, -1.8]
		point.velocities = [10.0, 10.0, 10.0, 10.0, 1.0]
		point.time_from_start = rospy.Duration(8.0)
		msg.points.append(point)
            	# publish joint stage message
            	self.arm_pub_1.publish(msg)	
		self.arm_pub_2.publish(msg)	
		#rospy.sleep(3)
		# Reverse turtlebot and open skirt door
		# How fast will we update the robot's movement? in Hz
        	rate = 20
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Set the forward linear speed to 0.1 meters per second
        	linear_speed = 0.1
        	# Set the travel distance in meters
        	goal_distance = 0.49
        	# Set the odom frame
        	self.odom_frame = 'turtlebot_1/odom'
		self.base_frame = 'turtlebot_1/base_footprint'
		# Initialize the position variable as a Point type
        	position_1 = Point()
		# Initialize the movement command
            	move_cmd_1 = Twist()
		# Set the movement command to reverse motion
            	move_cmd_1.linear.x = -linear_speed
            	# Get the starting position values
            	(position_1, rotation) = get_odom(self)        
            	x_start_1 = position_1.x
            	y_start_1 = position_1.y
            	# Keep track of the distance traveled
            	distance_1 = 0
		####################################################################################################
		# Set the odom frame
        	self.odom_frame = 'turtlebot_2/odom'
		self.base_frame = 'turtlebot_2/base_footprint'
		# Initialize the position variable as a Point type
        	position_2 = Point()
		# Initialize the movement command
            	move_cmd_2 = Twist()
		# Set the movement command to reverse motion
            	move_cmd_2.linear.x = -linear_speed
            	# Get the starting position values
            	(position_2, rotation) = get_odom(self)        
            	x_start_2 = position_2.x
            	y_start_2 = position_2.y
            	# Keep track of the distance traveled
            	distance_2 = 0
		rospy.loginfo("Begin moving turtlebot backwards")	
		# Enter the loop to move turtlebot
            	while distance_1 < goal_distance or distance_2 < goal_distance and not rospy.is_shutdown():
                	# Publish the Twist message and sleep 1 cycle
                	self.cmd_vel_1.publish(move_cmd_1)
			self.cmd_vel_2.publish(move_cmd_2)
                	# sleep
                	r.sleep()
                	# Get the current position
			# Set the odom frame
        		self.odom_frame = 'turtlebot_1/odom'
			self.base_frame = 'turtlebot_1/base_footprint'
                	(position_1, rotation_1) = get_odom(self)
			# Set the odom frame
        		self.odom_frame = 'turtlebot_2/odom'
			self.base_frame = 'turtlebot_2/base_footprint'
			(position_2, rotation_2) = get_odom(self)
                	# Compute the Euclidean distance from the start
                	distance_1 = sqrt(pow((position_1.x - x_start_1), 2) +
                                	pow((position_1.y - y_start_1), 2))
			distance_2 = sqrt(pow((position_2.x - x_start_2), 2) +
                                	pow((position_2.y - y_start_2), 2))
			# Check whether we need to stop one of the turtlebots early
			if distance_1 >= goal_distance:
				move_cmd_1 = Twist()
            			self.cmd_vel_1.publish(move_cmd_1)
			if distance_2 >= goal_distance:
				move_cmd_2 = Twist()
            			self.cmd_vel_2.publish(move_cmd_2)

		# Stop the robot for good
            	rospy.loginfo("Stop turtlebots")            	
		move_cmd_1 = Twist()
            	self.cmd_vel_1.publish(move_cmd_1)
		move_cmd_2 = Twist()
            	self.cmd_vel_2.publish(move_cmd_2)
            	rospy.sleep(1)

	def stage_7(self):

		# open gripper
		rospy.loginfo("Begin stage 7")
		# Create a joint state publisher
		self.joint5_pub_1 = rospy.Publisher('turtlebot_1/joint_5/command', Float64)
		self.joint5_pub_2 = rospy.Publisher('turtlebot_2/joint_5/command', Float64)
		rospy.sleep(1)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for joint 5, gripper to open
		rospy.loginfo("Position joint 5")
		msg = 0.1
            	# publish joint stage message
            	self.joint5_pub_1.publish(msg)
		self.joint5_pub_2.publish(msg)
		rospy.sleep(1)

	def stage_8(self):

		# move arms to stowed position
		rospy.loginfo("Begin stage 8")
		# Stow robotic arm
		# Create a joint state publisher for each turtlebot
		self.arm_1_pub = rospy.Publisher('turtlebot_1/arm_controller/command', JointTrajectory)
		self.arm_2_pub = rospy.Publisher('turtlebot_2/arm_controller/command', JointTrajectory)
		rospy.sleep(1)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for grasp position
		rospy.loginfo("Position arm in stowed position")
		rospy.sleep(3)
		msg = JointTrajectory()
		msg.header.seq = 1
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = '0'
		msg.joint_names.append('joint_1')
		msg.joint_names.append('joint_2')
		msg.joint_names.append('joint_3')
		msg.joint_names.append('joint_4')
		msg.joint_names.append('joint_5')
		# Define joint trajectory points
		point = JointTrajectoryPoint()
		point.positions  = [0.0, 1.5, 0.7, 0.0, -1.8]
		point.velocities = [2.0, 2.0, 20.0, 10.0, 1.0]
		point.time_from_start = rospy.Duration(2.0)
		msg.points.append(point)
		# Define joint trajectory points
		point = JointTrajectoryPoint()
		point.positions  = [0.0, -1.0, 1.9, 1.2, 0.1]
		point.velocities = [50.0, 50.0, 50.0, 50.0, 10.0]
		point.time_from_start = rospy.Duration(3.0)
		msg.points.append(point)
            	# publish joint stage message
            	self.arm_1_pub.publish(msg)
		self.arm_2_pub.publish(msg)
		rospy.sleep(3)

	def stage_9(self):

		# move single turtlebot to within inspection range
		rospy.loginfo("Begin stage 9")
		goal_angle = rospy.get_param("~goal_angle", radians(90)) 
		angular_speed = rospy.get_param("~angular_speed", 0.7) # radians per second
        	angular_tolerance = rospy.get_param("~angular_tolerance", radians(2)) # degrees to radians
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Publisher to control the robot's speed
     		self.cmd_vel = rospy.Publisher('turtlebot_2/cmd_vel_mux/input/navi', Twist)
		# Initialize the tf listener
        	self.tf_listener = tf.TransformListener()
		# Give tf some time to fill its buffer
        	rospy.sleep(1)
		# Set the odom frame
        	self.odom_frame = 'turtlebot_2/odom'
		self.base_frame = 'turtlebot_2/base_footprint'
		# Get the current rotation
                (position, rotation) = get_odom(self)
		# Initialize the movement command
            	move_cmd = Twist()
		# Set the movement command to a rotation
            	move_cmd.angular.z = angular_speed
            	# Track the last angle measured
            	last_angle = rotation
            	# Track how far we have turned
            	turn_angle = 0
		# Begin the rotation
            	while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
                	# Publish the Twist message and sleep 1 cycle
                	self.cmd_vel.publish(move_cmd)
			# sleep
			r.sleep()
                	# Get the current rotation
                	(position, rotation) = get_odom(self)
                	# Compute the amount of rotation since the last lopp
                	delta_angle = normalize_angle(rotation - last_angle)
                	turn_angle += delta_angle
                	last_angle = rotation
		# Stop the robot
            	rospy.loginfo("Stop turtlebot")            	
		move_cmd = Twist()
            	self.cmd_vel.publish(move_cmd)
		########################################################################################
		# move turtlebot forward
		# Set the forward linear speed to 0.1 meters per second
        	linear_speed = 0.1
        	# Set the travel distance in meters
        	goal_distance = 0.3
		# Set the odom frame
        	self.odom_frame = 'turtlebot_2/odom'
		self.base_frame = 'turtlebot_2/base_footprint'
		# Initialize the position variable as a Point type
        	position = Point()
		# Initialize the movement command
            	move_cmd = Twist()
            	# Set the movement command to forward motion
            	move_cmd.linear.x = linear_speed
            	# Get the starting position values
            	(position, rotation) = get_odom(self)      
            	x_start = position.x
            	y_start = position.y

            	# Keep track of the distance traveled
		distance = 0
		rospy.loginfo("Begin moving turtlebot forward")
		# Enter the loop to move turtlebot
            	while distance < goal_distance and not rospy.is_shutdown():
                	# Publish the Twist message and sleep 1 cycle
			self.cmd_vel.publish(move_cmd)
                	# sleep
                	r.sleep()
                	# Get the current position
			# Set the odom frame
        		self.odom_frame = 'turtlebot_2/odom'
			self.base_frame = 'turtlebot_2/base_footprint'
			(position, rotation) = get_odom(self)
                	# Compute the Euclidean distance from the start
			distance = sqrt(pow((position.x - x_start), 2) +
                                	pow((position.y - y_start), 2))
			# Check whether we need to stop one of the turtlebots early
			if distance >= goal_distance:
				move_cmd = Twist()
            			self.cmd_vel.publish(move_cmd)

		# Stop the robot before reversing
		rospy.loginfo("Stop turtlebot")            	
		move_cmd = Twist()
            	self.cmd_vel.publish(move_cmd)
            	rospy.sleep(1)
		###########################################################################################
		# rotate turtlebot in other direction
		goal_angle = rospy.get_param("~goal_angle", radians(90)) 		
		# Initialize the movement command
            	move_cmd = Twist()
		# Set the movement command to a rotation
            	move_cmd.angular.z = -angular_speed
            	# Track the last angle measured
            	last_angle = rotation
            	# Track how far we have turned
            	turn_angle = 0
		# Begin the rotation
            	while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
                	# Publish the Twist message and sleep 1 cycle
                	self.cmd_vel.publish(move_cmd)
			# sleep
			r.sleep()
                	# Get the current rotation
                	(position, rotation) = get_odom(self)
                	# Compute the amount of rotation since the last lopp
                	delta_angle = normalize_angle(rotation - last_angle)
                	turn_angle += delta_angle
                	last_angle = rotation
		# Stop the robot
            	rospy.loginfo("Stop turtlebot")            	
		move_cmd = Twist()
            	self.cmd_vel.publish(move_cmd)
		########################################################################################
		# move turtlebot forward
		# Set the forward linear speed to 0.1 meters per second
        	linear_speed = 0.1
        	# Set the travel distance in meters
        	goal_distance = 0.32
		# Set the odom frame
        	self.odom_frame = 'turtlebot_2/odom'
		self.base_frame = 'turtlebot_2/base_footprint'
		# Initialize the position variable as a Point type
        	position = Point()
		# Initialize the movement command
            	move_cmd = Twist()
            	# Set the movement command to forward motion
            	move_cmd.linear.x = linear_speed
            	# Get the starting position values
            	(position, rotation) = get_odom(self)      
            	x_start = position.x
            	y_start = position.y

            	# Keep track of the distance traveled
		distance = 0
		rospy.loginfo("Begin moving turtlebot forward")
		# Enter the loop to move turtlebot
            	while distance < goal_distance and not rospy.is_shutdown():
                	# Publish the Twist message and sleep 1 cycle
			self.cmd_vel.publish(move_cmd)
                	# sleep
                	r.sleep()
                	# Get the current position
			# Set the odom frame
        		self.odom_frame = 'turtlebot_2/odom'
			self.base_frame = 'turtlebot_2/base_footprint'
			(position, rotation) = get_odom(self)
                	# Compute the Euclidean distance from the start
			distance = sqrt(pow((position.x - x_start), 2) +
                                	pow((position.y - y_start), 2))
			# Check whether we need to stop one of the turtlebots early
			if distance >= goal_distance:
				move_cmd = Twist()
            			self.cmd_vel.publish(move_cmd)

		# Stop the robot before reversing
		rospy.loginfo("Stop turtlebot")            	
		move_cmd = Twist()
            	self.cmd_vel.publish(move_cmd)
            	rospy.sleep(1)



if __name__ == '__main__':
    try:
	# Initialise node	
	rospy.init_node('maintenance_planner', anonymous=False)
	# Declare maintenance planner
	planner = maintenance_planner()        
	planner.run()
    except Exception, e:
        print "Could not initiate maintenance planner", e
