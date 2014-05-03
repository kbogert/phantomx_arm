#! /usr/bin/env python

import rospy
# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus
# Import move base msgs for action and goal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Define class
class move_to_goal():
    def __init__(self, goal, ns):
	rospy.loginfo("Initialise move base action client")
	# Create a client using the simple action client, and subscribe to the move base goal server
	self.move_base = actionlib.SimpleActionClient(ns + '/move_base', MoveBaseAction)
	# Waits until the action server has started up and started listening for goals.
	self.move_base.wait_for_server()
	rospy.loginfo("Move Base is up")
	# Send the goal pose to the MoveBaseAction server
	self.move_base.send_goal(goal)
	rospy.loginfo("Goal sent")
	# wait for move base server to finish performing move action
	self.move_base.wait_for_result()
	rospy.loginfo("Result:")
	# Upon result, print outcome
	if self.move_base.get_state() == GoalStatus.SUCCEEDED:
		rospy.loginfo("Goal Achieved")
	else:
		rospy.logerr("Goal Failed")

if __name__ == '__main__':
    try:
        move_to_goal(self, goal)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Goal Achieved")
