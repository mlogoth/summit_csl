#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from math import atan2, sin, cos, sqrt


# Publisher
pub = rospy.Publisher('amcl_summit_pose', PoseStamped, queue_size=10)

def callback(data):
    msg = PoseStamped()
    msg.header = data.header
    msg.pose = data.pose.pose
    pub.publish(msg)

def listener():

    rospy.init_node('pwc2pose', anonymous=True)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
