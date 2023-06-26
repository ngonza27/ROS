#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point
import math
import tf

try:
    import thread
except ImportError:
    #for python3 compatability
    import _thread

class PathFollower:
    def __init__(self, path):
        self.path = path
        self.current_goal_index = 0
        self.current_goal = Point()
        self.odom = Odometry()
        self.sub_odom = rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odom_callback)
        self.pub_cmd = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=1)

    def odom_callback(self, odom):
        self.odom = odom
        self.update_current_goal()

    def get_yaw_from_quaternion(self, quaternion):
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        return tf.transformations.euler_from_quaternion(quaternion_list)[2]
    
    def update_current_goal(self):
        current_pos = self.odom.pose.pose.position
        if self.current_goal_index is 0:
            # search nearest point index
            dx = [current_pos.x - icx[0] for icx in self.path]
            dy = [current_pos.y - icy[1] for icy in self.path]
            d = math.sqrt((dx)**2 + (dy)**2)
            ind = np.argmin(d)
            self.current_goal_index = ind
        else:
            ind = self.current_goal_index
            distance = math.sqrt((current_pos.x - self.path[self.current_goal_index][0])**2 + (current_pos.y - self.path[self.current_goal_index][1])**2)
            while True:
                distance_nxt = math.sqrt((current_pos.x - self.path[self.current_goal_index+1][0])**2 + (current_pos.y - self.path[self.current_goal_index+1][1])**2)
                if distance < 0.5:
                    self.current_goal_index = ind
                    self.current_goal = Point(self.path[self.current_goal_index][0], self.path[self.current_goal_index][1], 0)
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance=distance_nxt

    def run(self):
        while not rospy.is_shutdown():
            current_pos = self.odom.pose.pose.position
            goal_pos = self.current_goal
            diff_x = goal_pos.x - current_pos.x
            diff_y = goal_pos.y - current_pos.y
            distance = math.sqrt(diff_x**2 + diff_y**2)
            if distance > 0.5:
                angle_to_goal = math.atan2(diff_y, diff_x)
                current_yaw = self.get_yaw_from_quaternion(self.odom.pose.pose.orientation)
                angle_diff = angle_to_goal - current_yaw
                if angle_diff > math.pi:
                    angle_diff -= 2*math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2*math.pi
                cmd_ack = AckermannDrive()
                cmd_ack.speed = 3.0
                cmd_ack.steering_angle = angle_diff*0.5
                self.pub_cmd.publish(cmd_ack)



if __name__ == '__main__':
    rospy.init_node('path_follower')
    path = [(41.461021423339844, -208.50003051757812), (41.461021423339844, -208.50003051757812), (41.461021423339844, -208.50003051757812), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46101379394531, -208.4999542236328), (41.46105194091797, -208.49993896484375), (41.46105194091797, -208.49993896484375), (41.46105194091797, -208.49993896484375), (41.46105194091797, -208.49993896484375), (41.46105194091797, -208.49993896484375), (41.46105194091797, -208.49993896484375), (41.46105194091797, -208.4999237060547), (41.46105194091797, -208.4999237060547), (41.46105194091797, -208.4999237060547), (41.46009826660156, -211.4820098876953), (41.46009826660156, -211.4820098876953), (41.46009826660156, -211.4820098876953), (41.459999084472656, -211.75289916992188), (41.459999084472656, -211.75289916992188), (41.459999084472656, -211.75289916992188), (41.45985412597656, -212.153076171875), (41.45985412597656, -212.153076171875), (41.45985412597656, -212.153076171875), (41.459754943847656, -212.41409301757812), (41.459754943847656, -212.41409301757812), (41.459754943847656, -212.41409301757812), (41.45814514160156, -216.04576110839844), (41.45814514160156, -216.04576110839844), (41.45814514160156, -216.04576110839844), (41.458045959472656, -216.26443481445312), (41.458045959472656, -216.26443481445312), (41.458045959472656, -216.26443481445312), (41.455116271972656, -222.55218505859375), (41.455116271972656, -222.55218505859375), (41.455116271972656, -222.55218505859375), (41.4780158996582, -224.49517822265625), (41.4780158996582, -224.49517822265625), (41.4780158996582, -224.49517822265625), (43.54472732543945, -235.71250915527344), (43.54472732543945, -235.71250915527344), (43.54472732543945, -235.71250915527344), (43.7010383605957, -236.1689910888672), (43.7010383605957, -236.1689910888672), (43.7010383605957, -236.1689910888672), (43.804710388183594, -236.47474670410156), (43.804710388183594, -236.47474670410156), (43.856571197509766, -236.62796020507812), (44.935585021972656, -240.2542266845703), (44.935585021972656, -240.2542266845703), (44.935585021972656, -240.2542266845703), (45.06403350830078, -240.7395782470703), (45.06403350830078, -240.7395782470703), (45.06403350830078, -240.7395782470703), (45.1467170715332, -241.064697265625), (45.1467170715332, -241.064697265625), (45.1467170715332, -241.064697265625), (45.62892150878906, -246.46192932128906)]
    follower = PathFollower(path)
    follower.run()
