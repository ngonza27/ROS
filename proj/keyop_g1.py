#!/usr/bin/env python
#This file goes inside /home/carla/Desktop/catkin_ws/src/av_carla_examples/scripts
'''
ackermann_drive_keyop.py:
    A ros keyboard teleoperation script for ackermann steering based robots
'''

__author__ = 'George Kouros'
__license__ = 'GPLv3'
__maintainer__ = 'George Kouros'
__email__ = 'gkourosg@yahoo.gr'

import roslib
import rospy
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleStatus
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import NavSatFix
#Package for the OxTS GPS
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import sys, select, termios, tty

try:
    import thread
except ImportError:
    #for python3 compatability
    import _thread
from numpy import clip

control_keys = {
    'up'    : '\x41',
    'down'  : '\x42',
    'right' : '\x43',
    'left'  : '\x44',
    'space' : '\x20',
    'tab'   : '\x09'}

key_bindings = {
    '\x41' : ( 1.0 , 0.0),
    '\x42' : (-1.0 , 0.0),
    '\x43' : ( 0.0 ,-1.0),
    '\x44' : ( 0.0 , 1.0),
    '\x20' : ( 0.0 , 0.0),
    '\x09' : ( 0.0 , 0.0)}

orientation=0
ego_latitude=0
ego_longitude=0
def gnss_callback(data):
    global ego_latitude, ego_longitude
    ego_latitude = data.latitude
    ego_longitude = data.longitude
   

def pose_callback(status):
    global orientation
    orientation = status.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])



class AckermannDriveKeyop:

    def __init__(self, args):
        if len(args) == 1:
            max_speed = float(args[0])
            max_steering_angle = float(args[0])
        elif len(args) >= 2:
            max_speed = float(args[0])
            max_steering_angle = float(args[1])
        else:
            max_speed = 0.2
            max_steering_angle = 0.7

        if len(args) > 2:
            cmd_topic = '/' + args[2]
        else:
            cmd_topic = 'ackermann_cmd'

        self.speed_range = [-float(max_speed), float(max_speed)]
        self.steering_angle_range = [-float(max_steering_angle),
                                     float(max_steering_angle)]
        for key in key_bindings:
            key_bindings[key] = \
                    (key_bindings[key][0] * float(max_speed) / 10,
                     key_bindings[key][1] * float(max_steering_angle) / 10)

        self.speed = 0
        self.steering_angle = 0
        self.motors_pub = rospy.Publisher(
            cmd_topic, AckermannDrive, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/5.0), self.pub_callback, oneshot=False)
        self.print_state()
        self.key_loop()

    def pub_callback(self, event):
        ackermann_cmd_msg = AckermannDrive()
        ackermann_cmd_msg.speed = 5
        ackermann_cmd_msg.steering_angle = self.steering_angle
        self.motors_pub.publish(ackermann_cmd_msg)

    def print_state(self):
        global orientation, ego_latitude, ego_longitude
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\rUse arrows to change speed and steering angle')
        rospy.loginfo('\x1b[1M\rUse space to brake and tab to align wheels')
        rospy.loginfo('\x1b[1M\rPress <ctrl-c> or <q> to exit')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteer Angle: \033[32;1m%0.2f rad\033[0m',
                      self.speed, self.steering_angle)
	#rospy.logwarn(orientation)
        rospy.logwarn(ego_latitude)
        rospy.logwarn(ego_longitude)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def key_loop(self):
        self.settings = termios.tcgetattr(sys.stdin)
        while 1:
            key = self.get_key()
            if key in key_bindings.keys():
                if key == control_keys['space']:
                    self.speed = 0.0
                elif key == control_keys['tab']:
                    self.steering_angle = 0.0
                else:
                    self.speed = self.speed + key_bindings[key][0]
                    self.steering_angle = \
                            self.steering_angle + key_bindings[key][1]
                    self.speed = clip(
                        self.speed, self.speed_range[0], self.speed_range[1])
                    self.steering_angle = clip(
                        self.steering_angle,
                        self.steering_angle_range[0],
                        self.steering_angle_range[1])
                self.print_state()
            elif key == '\x03' or key == '\x71':  # ctr-c or q
                break
            else:
                continue
        self.finalize()

    def finalize(self):
        rospy.loginfo('Halting motors, aligning wheels and exiting...')
        self.settings = termios.tcgetattr(sys.stdin)
        ackermann_cmd_msg = AckermannDrive()
        ackermann_cmd_msg.drive.speed = 0
        ackermann_cmd_msg.drive.steering_angle = 0
        self.motors_pub.publish(ackermann_cmd_msg)
        sys.exit()

if __name__ == '__main__':
    rospy.init_node('ackermann_drive_keyop_node')
    sub=rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, callback=pose_callback)
    sub1=rospy.Subscriber("/carla/ego_vehicle/gnss/default/fix", NavSatFix, callback=gnss_callback)
    keyop = AckermannDriveKeyop(sys.argv[1:len(sys.argv)])
