lat=0
lon=0
ang=0
i=0

def odometry_callback(odometry_msg):
    global lat,lon,ang
    lat = odometry_msg.pose.pose.position.x
    lon = odometry_msg.pose.pose.position.y

    q_x = odometry_msg.pose.pose.orientation.x
    q_y = odometry_msg.pose.pose.orientation.y
    q_z = odometry_msg.pose.pose.orientation.z
    q_w = odometry_msg.pose.pose.orientation.w

    ang = tf.transformations.euler_from_quaternion((q_x, q_y, q_z, q_w))[2]

    # Wrap yaw to 0-360 degrees
    if ang < 0:
        ang += 360
#!/usr/bin/env python
#This file goes inside /home/carla/Desktop/catkin_ws/src/av_carla_examples/scripts
'''
ackermann_drive_keyop.py:
    A ros keyboard teleoperation script for ackermann steering based robots
'''

import roslib
import rospy
import math
import tf
import numpy as np
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleStatus
from sensor_msgs.msg import NavSatFix
#START -- Package for the OxTS GPS
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
#END ---
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

Ks=0.9
lat=0
lon=0
ang=0
i=0

def odometry_callback(odometry_msg):
    global lat,lon,ang
    lat = odometry_msg.pose.pose.position.x
    lon = odometry_msg.pose.pose.position.y

    q_x = odometry_msg.pose.pose.orientation.x
    q_y = odometry_msg.pose.pose.orientation.y
    q_z = odometry_msg.pose.pose.orientation.z
    q_w = odometry_msg.pose.pose.orientation.w

    ang = tf.transformations.euler_from_quaternion((q_x, q_y, q_z, q_w))[2]

    # Wrap yaw to 0-360 degrees
    if ang < 0:
        ang += 360


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
        
        self.coords = [[41.51051330566406, -226.03700256347656], [41.51051330566406, -226.03700256347656], [41.51051330566406, -226.03700256347656], [41.55595779418945, -227.0623779296875], [41.55595779418945, -227.0623779296875], [41.56186294555664, -227.17703247070312], [41.58065414428711, -227.5218048095703], [41.58065414428711, -227.5218048095703], [41.58065414428711, -227.5218048095703], [41.62558364868164, -228.09722900390625], [41.62558364868164, -228.09722900390625], [41.63581848144531, -228.21249389648438], [41.680145263671875, -228.67404174804688], [41.680145263671875, -228.67404174804688], [41.680145263671875, -228.67404174804688], [41.770809173583984, -229.48193359375], [41.770809173583984, -229.48193359375], [41.770809173583984, -229.48193359375], [41.8311653137207, -229.94302368164062], [41.8311653137207, -229.94302368164062], [41.8311653137207, -229.94302368164062], [42.030696868896484, -231.0893096923828], [42.030696868896484, -231.0893096923828], [42.030696868896484, -231.0893096923828], [42.17527770996094, -231.7712860107422], [42.17527770996094, -231.7712860107422], [42.17527770996094, -231.7712860107422], [42.31924057006836, -232.3320770263672], [42.31924057006836, -232.3320770263672], [42.31924057006836, -232.3320770263672], [42.446083068847656, -232.77735900878906], [42.446083068847656, -232.77735900878906], [42.446083068847656, -232.77735900878906], [42.479373931884766, -232.8882293701172], [42.479373931884766, -232.8882293701172], [42.513362884521484, -232.99899291992188], [42.58326721191406, -233.2200927734375], [42.58326721191406, -233.2200927734375], [42.58326721191406, -233.2200927734375], [42.77416229248047, -233.76841735839844], [42.77416229248047, -233.76841735839844], [42.81560134887695, -233.877197265625], [42.85781478881836, -233.98577880859375], [42.85781478881836, -233.98577880859375], [42.85781478881836, -233.98577880859375], [43.03619384765625, -234.418212890625], [43.03619384765625, -234.418212890625], [43.03619384765625, -234.418212890625], [43.28740310668945, -234.9501190185547], [43.28740310668945, -234.9501190185547], [43.28740310668945, -234.9501190185547], [43.50690841674805, -235.37118530273438], [43.50690841674805, -235.37118530273438], [43.56431198120117, -235.47573852539062], [43.94025802612305, -236.08721923828125], [43.94025802612305, -236.08721923828125], [43.94025802612305, -236.08721923828125], [44.36842346191406, -236.66896057128906], [44.36842346191406, -236.66896057128906], [44.36842346191406, -236.66896057128906], [44.682743072509766, -237.03561401367188], [44.682743072509766, -237.03561401367188], [44.682743072509766, -237.03561401367188], [45.46206283569336, -237.79759216308594], [45.46206283569336, -237.79759216308594], [45.46206283569336, -237.79759216308594], [45.74136734008789, -238.03050231933594], [45.74136734008789, -238.03050231933594], [45.74136734008789, -238.03050231933594], [46.52720642089844, -238.59446716308594], [46.52720642089844, -238.59446716308594], [46.52720642089844, -238.59446716308594], [47.57958221435547, -239.17518615722656], [47.57958221435547, -239.17518615722656], [47.57958221435547, -239.17518615722656], [48.019222259521484, -239.3676300048828], [48.019222259521484, -239.3676300048828], [48.019222259521484, -239.3676300048828], [48.69587326049805, -239.61251831054688], [48.69587326049805, -239.61251831054688], [48.69587326049805, -239.61251831054688], [49.738739013671875, -239.8779754638672], [49.738739013671875, -239.8779754638672], [49.85597229003906, -239.89984130859375], [49.85597229003906, -239.89984130859375], [49.85597229003906, -239.89984130859375], [49.85597229003906, -239.89984130859375], [51.03388595581055, -240.03335571289062], [51.03388595581055, -240.03335571289062], [51.03388595581055, -240.03335571289062], [51.03388595581055, -240.03335571289062], [51.03388595581055, -240.03335571289062], [51.03388595581055, -240.03335571289062], [51.50402069091797, -240.0438995361328], [52.20380401611328, -240.01934814453125], [52.20380401611328, -240.01934814453125], [52.20380401611328, -240.01934814453125], [52.20380401611328, -240.01934814453125], [52.20380401611328, -240.01934814453125], [52.20380401611328, -240.01934814453125], [53.3565559387207, -240.09165954589844], [53.3565559387207, -240.09165954589844], [53.3565559387207, -240.09165954589844], [53.92723846435547, -240.1297149658203], [53.92723846435547, -240.1297149658203], [53.92723846435547, -240.1297149658203], [54.49437713623047, -240.16761779785156], [54.49437713623047, -240.16761779785156], [54.49437713623047, -240.16761779785156], [55.62031936645508, -240.24281311035156], [55.62031936645508, -240.24281311035156], [55.62031936645508, -240.24281311035156], [56.17951202392578, -240.2800750732422], [56.17951202392578, -240.2800750732422], [56.29109573364258, -240.28749084472656], [56.73728561401367, -240.3171844482422], [56.73728561401367, -240.3171844482422], [56.73728561401367, -240.3171844482422], [57.18360900878906, -240.34686279296875], [57.18360900878906, -240.34686279296875], [57.18360900878906, -240.34686279296875], [57.85500717163086, -240.3916473388672], [57.85500717163086, -240.3916473388672], [57.85500717163086, -240.3916473388672], [58.64338684082031, -240.4442901611328], [58.64338684082031, -240.4442901611328], [58.64338684082031, -240.4442901611328], [59.09664535522461, -240.4745330810547], [59.09664535522461, -240.4745330810547], [59.09664535522461, -240.4745330810547], [60.008426666259766, -240.53546142578125], [60.008426666259766, -240.53546142578125], [60.008426666259766, -240.53546142578125]]
        self.speed = 0
        self.target_speed =  10.0 / 3.6 #10km/h
        self.steering_angle = 0
        self.old_nearest_point_index = 0
        self.motors_pub = rospy.Publisher(
            cmd_topic, AckermannDrive, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/5.0), self.pub_callback, oneshot=False)
        #self.print_state()
        self.key_loop()

    def pub_callback(self, event):
        ackermann_cmd_msg = AckermannDrive()
        ackermann_cmd_msg.speed = self.speed
        ackermann_cmd_msg.steering_angle = self.steering_angle
        self.motors_pub.publish(ackermann_cmd_msg)

    def closest_point(self,ref_x, ref_y, points):
        ref_point = np.array([ref_x, ref_y])
        points = np.array(points)
        distances = np.sqrt(np.sum((points - ref_point)**2, axis=1))
        return np.argmin(distances)

    def proportional_control(self,target, current):
        return  Ks * (target - current)

    def calc_distance(self, point_x, point_y):
        dx = point_x-lat 
        dy = point_y-lon 
        return math.hypot(dx, dy)-ang

    def move_car(self, x_goal, y_goal):
        global lat,lon,ang,i
        while (True):
            rate = rospy.Rate(100)
            KP=0.21
            distance = math.sqrt(math.pow(x_goal-lat,2)+math.pow(y_goal-lon,2))
            rospy.logwarn(distance)
            rospy.logwarn(i)
            
            desired_angle_goal = math.atan2(y_goal-lon, x_goal-lat)
            del_y = distance * math.sin(desired_angle_goal-ang)
            sign = 1 if x_goal - lat < 0 else -1
            curvature = sign * 2.0 * del_y / (distance ** 2)
            control_ang = KP * curvature
            control_ang = max(min(control_ang, 0.7), -0.7)
            car_speed = self.proportional_control(self.target_speed, self.speed)
            rospy.logerr(car_speed)
            self.speed = car_speed 
            self.steering_angle = control_ang
            rate.sleep()
            if distance<0.4:
              i += 1
              break
            elif distance>3:
                i = self.old_nearest_point_index
                distance_this_index = self.calc_distance(self.coords[i][0],self.coords[i][1])
                while True:
                    if (i+1)>=len(self.coords):
                        self.finalize()
                        break
                    distance_next_index = self.calc_distance(self.coords[i+1][0],self.coords[i+1][1])
                    if distance_this_index < distance_next_index:
                        break
                    i = i + 1 if (i + 1) < len(self.coords) else i
                    distance_this_index = distance_next_index
                self.old_nearest_point_index = i
                break

    def print_state(self):
        global lat,lon,ang
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
        rospy.logwarn(lat)
        rospy.logwarn(lon)
        rospy.logwarn(ang)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def key_loop(self):
        global i
        self.settings = termios.tcgetattr(sys.stdin)
        while i <= len(self.coords)+1:
            self.steering = self.move_car(self.coords[i][0], self.coords[i][1])
            #self.print_state()
            #if key == '\x03' or key == '\x71':  # ctr-c or q
             #   break
            #else:
            continue
        self.finalize()

    def finalize(self):
        rospy.loginfo('Halting motors, aligning wheels and exiting...')
        self.settings = termios.tcgetattr(sys.stdin)
        ackermann_cmd_msg = AckermannDrive()
        ackermann_cmd_msg.speed = 0
        ackermann_cmd_msg.steering_angle = 0
        self.motors_pub.publish(ackermann_cmd_msg)
        #sys.exit()

if __name__ == '__main__':
    rospy.init_node('ackermann_drive_keyop_node')
    sub=rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, callback=odometry_callback)
    keyop = AckermannDriveKeyop(sys.argv[1:len(sys.argv)])

