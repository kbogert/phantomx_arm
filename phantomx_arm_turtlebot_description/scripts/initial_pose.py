#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *

def gen_pose(msg):
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped)
    rospy.loginfo("Setting Pose")
    # pass initial pose to publisher and publish
    pose = PoseWithCovarianceStamped();
    pose.pose = msg;
    pub.publish(pose);
    rospy.loginfo("initial pose set")

if __name__ == '__main__':
    try:
        gen_pose(msg)
        rospy.loginfo("initial pose set")
    except Exception, e:
        print "error: ", e
