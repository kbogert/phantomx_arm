#! /usr/bin/env python

# Import 
import rospy
import tf
from utils.transform_utils import quat_to_angle
from geometry_msgs.msg import Point, Quaternion

def get_odom(self):
    	# Get the current transform between the odom and base frames	
	try:
            	(trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            	rospy.loginfo("TF Exception")
            	return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

if __name__ == '__main__':
    try:
        get_odom()
        rospy.loginfo("navigation action")
    except Exception, e:
        print "error: ", e
