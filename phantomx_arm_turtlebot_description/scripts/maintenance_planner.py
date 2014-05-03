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
from utils.navigation import *
from math import sqrt, pow
import tf

class maintenance_planner():
    	def run(self):
		# Generate starting position of first turtlebot
    		msg = PoseWithCovariance();
    		msg.pose = Pose(Point(0.058769719, -0.4287806, 0.000), Quaternion(0.000, 0.000, 0.1755776757, 0.9844655));
    		msg.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
		initial_pose.gen_pose(msg)	
		# Stow robotic arm
		# Create a joint state publisher
		self.arm_pub = rospy.Publisher('arm_controller/command', JointTrajectory)
		rospy.sleep(1)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for grasp position
		rospy.loginfo("Position arm in stowed position")
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
            	self.arm_pub.publish(msg)
		rospy.sleep(3)

		# Run each stage of maintenance
		self.stage_3()
		#self.stage_4()
		#self.stage_5()
		#self.stage_6()
		#self.stage_7()

	def stage_1(self):

		# Move to initial position using move base goal
		rospy.loginfo("Begin stage 1")
		# Move turtlebot to starting position
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
		move_to_goal.move_to_goal(goal)

	def stage_2(self):

		# Move to initial position using move base goal
		rospy.loginfo("Begin stage 2")

	def stage_3(self):

		# Move arm to grasping position
		rospy.loginfo("Begin stage 3")
		# Create a joint state publisher
		self.arm_pub = rospy.Publisher('arm_controller/command', JointTrajectory)
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
            	self.arm_pub.publish(msg)	
		rospy.sleep(3)

	def stage_4(self):

		# Move turtlebot to grasping position
		rospy.loginfo("Begin stage 4")
		# Publisher to control the robot's speed
     		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist)
		# How fast will we update the robot's movement? in Hz
        	rate = 20
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Set the forward linear speed to 0.1 meters per second
        	linear_speed = 0.1
        	# Set the travel distance in meters
        	goal_distance = 1.05
		# Initialize the tf listener
        	self.tf_listener = tf.TransformListener()
        	# Give tf some time to fill its buffer
        	rospy.sleep(2)
        	# Set the odom frame
        	self.odom_frame = '/odom'
		# Find out if the robot uses /base_link or /base_footprint
        	try:
            		self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            		self.base_frame = '/base_footprint'
			rospy.loginfo("base frame: /base_footprint")
        	except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            		try:
                		self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                		self.base_frame = '/base_link'
				rospy.loginfo("base frame: /base_link")
            		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                		rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                		rospy.signal_shutdown("tf Exception") 
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
		# Enter the loop to move along a side
            	while distance < goal_distance and not rospy.is_shutdown():
                	# Publish the Twist message and sleep 1 cycle
                	self.cmd_vel.publish(move_cmd)
                	# sleep
                	r.sleep()
                	# Get the current position
                	(position, rotation) = get_odom(self)
                	# Compute the Euclidean distance from the start
                	distance = sqrt(pow((position.x - x_start), 2) +
                                	pow((position.y - y_start), 2))

		# Stop the robot before reversing
		rospy.loginfo("Stop turtlebot")            	
		move_cmd = Twist()
            	self.cmd_vel.publish(move_cmd)
            	rospy.sleep(1)

	def stage_5(self):

		# Close gripper
		rospy.loginfo("Begin stage 5")
		# Create a joint state publisher
		self.joint5_pub = rospy.Publisher('joint_5/command', Float64)
		rospy.sleep(1)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for joint 5, gripper to close
		rospy.loginfo("Position joint 5")
		msg = -1.8
            	# publish joint stage message
            	self.joint5_pub.publish(msg)
		rospy.sleep(1)

	def stage_6(self):

		# Reverse turtlebot a specfic distance and lower arm at a specific velocity to target position
		rospy.loginfo("Begin stage 6")
		# Publisher to control the robot's speed
     		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist)
		# Create a joint state publisher
		self.arm_pub = rospy.Publisher('arm_controller/command', JointTrajectory)
		# Initialize the tf listener
        	self.tf_listener = tf.TransformListener()
        	# Give tf some time to fill its buffer
        	rospy.sleep(2)
		# Find out if the robot uses /base_link or /base_footprint
        	try:
            		self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            		self.base_frame = '/base_footprint'
			rospy.loginfo("base frame: /base_footprint")
        	except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            		try:
                		self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                		self.base_frame = '/base_link'
				rospy.loginfo("base frame: /base_link")
            		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                		rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                		rospy.signal_shutdown("tf Exception") 
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
		point.positions  = [0.0, 0.0, -1.0, -1.0, -1.8]
		point.velocities = [20.0, 20.0, 20.0, 20.0, 10.0]
		point.time_from_start = rospy.Duration(1.0)
		msg.points.append(point)
		# Define joint trajectory points
		point = JointTrajectoryPoint()
		point.positions  = [0.0, 1.5, 0.0, 0.0, -1.8]
		point.velocities = [10.0, 10.0, 10.0, 10.0, 10.0]
		point.time_from_start = rospy.Duration(2.0)
		msg.points.append(point)
            	# publish joint stage message
            	self.arm_pub.publish(msg)	
		#rospy.sleep(3)
		# Reverse turtlebot and open skirt door
		# How fast will we update the robot's movement? in Hz
        	rate = 20
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Set the forward linear speed to 0.1 meters per second
        	linear_speed = 0.15
        	# Set the travel distance in meters
        	goal_distance = 0.40
        	# Set the odom frame
        	self.odom_frame = '/odom'
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
		rospy.loginfo("Begin moving turtlebot backwards")
		# Set the movement command to reverse motion
            	move_cmd.linear.x = -linear_speed
            	# Get the starting position values
            	(position, rotation) = get_odom(self)        
            	x_start = position.x
            	y_start = position.y
            	# Keep track of the distance traveled
            	distance = 0

		# Enter the loop to move along a side
            	while distance < goal_distance and not rospy.is_shutdown():
                	# Publish the Twist message and sleep 1 cycle
                	self.cmd_vel.publish(move_cmd)
                	# sleep
                	r.sleep()
                	# Get the current position
                	(position, rotation) = get_odom(self)
                	# Compute the Euclidean distance from the start
                	distance = sqrt(pow((position.x - x_start), 2) +
                                	pow((position.y - y_start), 2))

		# Stop the robot for good
            	move_cmd = Twist()
            	self.cmd_vel.publish(move_cmd)
            	rospy.sleep(1)

	def stage_7(self):

		# open gripper
		rospy.loginfo("Begin stage 5")
		# Create a joint state publisher
		self.joint5_pub = rospy.Publisher('joint_5/command', Float64)
		rospy.sleep(1)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for joint 5, gripper to open
		rospy.loginfo("Position joint 5")
		msg = 0.1
            	# publish joint stage message
            	self.joint5_pub.publish(msg)
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
