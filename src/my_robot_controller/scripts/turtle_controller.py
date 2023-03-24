#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def pose_callback(pose: Pose):
	cmd=Twist()
	if pose.x > 9.0:
		cmd.linear.x=1.0
		cmd.angular.z=1.4
	else:
		cmd.linear.x=5.0
		cmd.angular.z=0.0
	pub.publish(cmd)

if __name__ == '__main__':
	rospy.init_node("turtle_pose_subscriber")
	pub=rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
	sub=rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
	rospy.loginfo("Node has been started")
	rospy.spin()