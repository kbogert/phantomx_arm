#! /usr/bin/env python

# Import
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import *
from std_msgs.msg import *
from utils.navigation import *
from math import sqrt, pow
import tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import move_to_goal

class test():
    	def __init__(self):
		# Run tests
		self.test_4()
		#self.test_6()
		#self.test_7()	

	def test_1(self):

		# Test 1: move turtlebot forwards and backwards at a specific velocity using cmd_vel topic
		rospy.loginfo("Begin test 1")
		# Publisher to control the robot's speed
     		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist)
		# How fast will we update the robot's movement? in Hz
        	rate = 20
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Set the forward linear speed to 0.1 meters per second
        	linear_speed = 0.1
        	# Set the travel distance in meters
        	goal_distance = 1.0
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
		# End of test 1
		rospy.loginfo("End test 1")

	def test_2(self):
	
		# Test 2: move turtlebot to a designated spot using amcl, map and move_base topic
		rospy.loginfo("Begin test 2")
		# Intialize the waypoint goal
            	goal = MoveBaseGoal()
            	# Use the map frame to define goal poses
            	goal.target_pose.header.frame_id = 'map'
		rospy.loginfo("Define position relative to map")
            	# Set pose
        	goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = 0.840401411057
        	goal.target_pose.pose.position.y = 0.674615263939
        	goal.target_pose.pose.orientation = Quaternion(0.0,0.0,-0.631627191685, 0.77527226877)
		# Move to goal
		move_to_goal.move_to_goal(goal)
		rospy.loginfo("End test 2")

	def test_3(self):

		# Test 3: move each joint in robotic arm (medium speed)
		rospy.loginfo("Begin test 3")
		# Create a joint state publisher
		self.joint1_pub = rospy.Publisher('joint_1/command', Float64)
		self.joint2_pub = rospy.Publisher('joint_2/command', Float64)
		self.joint3_pub = rospy.Publisher('joint_3/command', Float64)
		self.joint4_pub = rospy.Publisher('joint_4/command', Float64)
		self.joint5_pub = rospy.Publisher('joint_5/command', Float64)
		rospy.sleep(1)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for joint 1
		rospy.loginfo("Position joint 1")
		msg = -1.0
		rospy.sleep(1)
            	# publish joint stage message
            	self.joint1_pub.publish(msg)
		rospy.sleep(1)
		# Define a joint trajectory message for joint 2
		rospy.loginfo("Position joint 2")
		msg = 0.5
		rospy.sleep(1)
            	# publish joint stage message
            	self.joint2_pub.publish(msg)
		rospy.sleep(1)
		# Define a joint trajectory message for joint 3
		rospy.loginfo("Position joint 3")
		msg = -1.0
		rospy.sleep(1)
            	# publish joint stage message
            	self.joint3_pub.publish(msg)
		# Define a joint trajectory message for joint 3
		rospy.loginfo("Position joint 4")
		msg = 0.5
		rospy.sleep(1)
            	# publish joint stage message
            	self.joint4_pub.publish(msg)
		# Define a joint trajectory message for joint 3
		rospy.loginfo("Position joint 5")
		msg = -1.0
		rospy.sleep(1)
            	# publish joint stage message
            	self.joint5_pub.publish(msg)
		rospy.sleep(1)
		rospy.loginfo("End test 3")

	def test_4(self):

		# Test 4: move each joint in robotic arm to a specific value at the same time (slow speed)
		rospy.loginfo("Begin test 4")
		# Create a joint state publisher
		self.arm_pub = rospy.Publisher('arm_controller/command', JointTrajectory)
		rospy.sleep(1)
		# How fast will we update the robot's movement? in Hz
        	rate = 10
		# Set the equivalent ROS rate variable
        	r = rospy.Rate(rate)
		# Define a joint trajectory message for stand up straight
		rospy.loginfo("Position arm in straight position")
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
		point.positions  = [0.0, 0.0, 0.0, 0.6, -1.8]
		point.velocities = [10.0, 10.0, 10.0, 10.0, 10.0]
		point.time_from_start = rospy.Duration(3.0)
		msg.points.append(point)
            	# publish joint stage message
            	self.arm_pub.publish(msg)
		rospy.loginfo("End test 4")

	def test_5(self):

		# Test 5: move each joint in robotic arm to a specific value at the same time (mixed speeds)
		rospy.loginfo("Begin test 5")
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
		point.velocities = [90.0, 50.0, 20.0, 10.0, 10.0]
		point.time_from_start = rospy.Duration(3.0)
		msg.points.append(point)
            	# publish joint stage message
            	self.arm_pub.publish(msg)
		rospy.loginfo("End test 5")

if __name__ == '__main__':
    try:
	# Initialise node	
	rospy.init_node('action_test', anonymous=False)
	# Declare action test
	test()        
    except Exception, e:
        print "Could not initiate action test", e
