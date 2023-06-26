#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point
import math
import tf
import time
import random

class PathFollower:
    def __init__(self, path):
        self.path = path
        self.current_goal_index = 0
        self.current_goal = Point()
        self.odom = Odometry()
        self.sub_odom = rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odom_callback)
        self.pub_cmd = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=1)
	self.car_coords=[]

    def odom_callback(self, odom):
        self.odom = odom
        self.update_current_goal()

    def finalize(self):
        rospy.loginfo('Halting motors, aligning wheels and exiting...')
        ackermann_cmd_msg = AckermannDrive()
        ackermann_cmd_msg.speed = 0
        ackermann_cmd_msg.steering_angle = 0
        self.pub_cmd.publish(ackermann_cmd_msg)
	rospy.loginfo(self.car_coords)

    def update_current_goal(self):
        current_pos = self.odom.pose.pose.position
        goal_pos = Point(self.path[self.current_goal_index][0], self.path[self.current_goal_index][1], 0)
        distance = math.sqrt((current_pos.x - goal_pos.x)**2 + (current_pos.y - goal_pos.y)**2)
        angle_to_goal = math.atan2(current_pos.y - goal_pos.y, current_pos.x - goal_pos.x)

        while angle_to_goal < 0:
	    if self.current_goal_index >= len(self.path)-1:
	        self.finalize()
            self.current_goal_index = self.current_goal_index + 1 if (self.current_goal_index + 1) < len(self.path) else self.current_goal_index
	    goal_pos = Point(self.path[self.current_goal_index][0], self.path[self.current_goal_index][1], 0)
	    angle_to_goal = math.atan2(current_pos.y - goal_pos.y, current_pos.x - goal_pos.x)
            if angle_to_goal >= 0:
	        break
        if distance < 0.5:
            if self.current_goal_index < len(self.path)-1:
                self.current_goal_index += 1
                self.current_goal = Point(self.path[self.current_goal_index][0], self.path[self.current_goal_index][1], 0)
	    else:
	        self.finalize()
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            current_pos = self.odom.pose.pose.position
            goal_pos = self.current_goal
	    if goal_pos.x == 0:
                goal_pos = Point(self.path[self.current_goal_index][0], self.path[self.current_goal_index][1], 0)
            diff_x = goal_pos.x - current_pos.x
            diff_y = goal_pos.y - current_pos.y
            distance = math.sqrt(diff_x**2 + diff_y**2)
            if distance > 0.5:
                angle_to_goal = math.atan2(diff_y, diff_x)
                current_yaw = self.get_yaw_from_quaternion(self.odom.pose.pose.orientation)
                angle_diff = angle_to_goal - current_yaw
                rospy.logwarn(self.current_goal_index)
                rospy.loginfo(len(self.path))
                rospy.logwarn(goal_pos)
                if angle_diff > math.pi:
                    angle_diff -= 2*math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2*math.pi
                cmd_ack = AckermannDrive()
                cmd_ack.speed = 4.0
                cmd_ack.steering_angle = max(min(angle_diff*0.5, 0.7), -0.7)
                self.pub_cmd.publish(cmd_ack)
            else:
                cmd_ack = AckermannDrive()
                cmd_ack.speed = 2.0
                cmd_ack.steering_angle = 0.0
                self.pub_cmd.publish(cmd_ack)
            rate.sleep()
            self.car_coords.append([current_pos.x,current_pos.y])

    def get_yaw_from_quaternion(self, quaternion):
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        return tf.transformations.euler_from_quaternion(quaternion_list)[2]


if __name__ == '__main__':
    rospy.init_node('path_follower')
    path = [(41.461021423339844, -208.50003051757812), (41.461021423339844, -208.50003051757812), (41.461021423339844, -208.50003051757812), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46105194091797, -208.49993896484375), (41.46105194091797, -208.49993896484375), (41.46105194091797, -208.49993896484375), (41.46105194091797, -208.49993896484375), (41.46105194091797, -208.49993896484375), (41.46105194091797, -208.49993896484375), (41.46105194091797, -208.4999237060547), (41.46105194091797, -208.4999237060547), (41.46105194091797, -208.4999237060547), (41.46009826660156, -211.4820098876953), (41.46009826660156, -211.4820098876953), (41.46009826660156, -211.4820098876953), (41.459999084472656, -211.75289916992188), (41.459999084472656, -211.75289916992188), (41.459999084472656, -211.75289916992188), (41.45985412597656, -212.153076171875), (41.45985412597656, -212.153076171875), (41.45985412597656, -212.153076171875), (41.459754943847656, -212.41409301757812), (41.459754943847656, -212.41409301757812), (41.459754943847656, -212.41409301757812), (41.45814514160156, -216.04576110839844), (41.45814514160156, -216.04576110839844), (41.45814514160156, -216.04576110839844), (41.458045959472656, -216.26443481445312), (41.458045959472656, -216.26443481445312), (41.458045959472656, -216.26443481445312), (41.455116271972656, -222.55218505859375), (41.455116271972656, -222.55218505859375), (41.455116271972656, -222.55218505859375), (41.4780158996582, -224.49517822265625), (41.4780158996582, -224.49517822265625), (41.4780158996582, -224.49517822265625), (43.54472732543945, -235.71250915527344), (43.54472732543945, -235.71250915527344), (43.54472732543945, -235.71250915527344), (43.7010383605957, -236.1689910888672), (43.7010383605957, -236.1689910888672), (43.7010383605957, -236.1689910888672), (43.804710388183594, -236.47474670410156), (43.804710388183594, -236.47474670410156), (43.856571197509766, -236.62796020507812), (44.935585021972656, -240.2542266845703), (44.935585021972656, -240.2542266845703), (44.935585021972656, -240.2542266845703), (45.06403350830078, -240.7395782470703), (45.06403350830078, -240.7395782470703), (45.06403350830078, -240.7395782470703), (45.1467170715332, -241.064697265625), (45.1467170715332, -241.064697265625), (45.1467170715332, -241.064697265625), (45.62892150878906, -246.46192932128906)]
    
    follower = PathFollower(path)
    follower.run()
