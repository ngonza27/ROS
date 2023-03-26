#!/usr/bin/env python3
#This code is currently doing pub/sub to the turtlesim package from ROS
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

x=0
y=0
yaw=0

def pose_callback(pose_message):
    global x
    global y, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta


def go_to_goal(x_goal, y_goal):
    global x
    global y, yaw	

    velocity_message = Twist()
    velocity_publisher=rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    while (True):
        #K_linear = 0.5 
        K_linear = 1
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        #Speed control
        #linear_speed = distance * K_linear 
        linear_speed = K_linear

        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        #Angle control
        angular_speed = (desired_angle_goal-yaw)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        #print(f"x={x}, y={y}, desired__heading_angle={desired_angle_goal}, current_heading_angle={yaw}, control_angle={desired_angle_goal-yaw}")
        print(f"x={x:.2f}, y={y:.2f}, dsrd__hdng_ang={desired_angle_goal:.2f}, curr_hdng_ang={yaw:.2f}, ctrl_ang={(desired_angle_goal-yaw):.2f}")
        
        if (distance <0.01):
            break


if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlebot_move_gdp')
        rospy.loginfo(f"Node (turtlebot_move_gdp) has been started")
        sub=rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
       	go_to_goal(9, 9)

        coords=[[8.682047169799638, 5.480804324509112], [8.68203496180451, 5.480802243586227], [8.682022215162757, 5.480800527737437], [8.682012206314312, 5.480799542037071], [8.68200080605659, 5.480798592844133], [8.681974549376555, 5.480801038841207], [8.681943220562756, 5.480804397523913], [8.681922169911047, 5.480806332417298], [8.681893264274338, 5.4808060768653], [8.68187562450383, 5.480806551461731], [8.681867455411162, 5.4808059673429925], [8.681858927211636, 5.480802681675282], [8.681853810278803, 5.480797899203143], [8.681850174557766, 5.480791985001264], [8.681843486613722, 5.48077325669422], [8.681839491793593, 5.480758507695308], [8.681837427052892, 5.480750476063126], [8.681834599253309, 5.480741969834497], [8.681832893594999, 5.480733828680058], [8.681830469762938, 5.480723314543383], [8.681829811111111, 5.480718252777778]]
        for i in coords:
            go_to_goal(i[0], i[1])

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")


# coords=[[52.069862778539544, -0.628222106974543],
# [52.06966301134649, -0.6282902099052774],
# [52.06945442993601, -0.6283463649565979],
# [52.069290648779635, -0.6283786242412852],
# [52.069104099107854, -0.6284096887374696],
# [52.068674444343635, -0.6283296379241499],
# [52.068161791026895, -0.6282197173991845],
# [52.067817325817124, -0.628156393615691],
# [52.06734432448916, -0.6281647571356348],
# [52.06705567369904, -0.6281492248888229],
# [52.06692199763719, -0.6281683415020566],
# [52.06678244528129, -0.6282758724453217],
# [52.066698713653125, -0.6284323897152961],
# [52.0666392200362, -0.6286259454131731],
# [52.06652978095183, -0.6292388718255206],
# [52.0664644111679, -0.6297215663353769],
# [52.066430624501855, -0.629984419752247],
# [52.06638435141778, -0.6302628054164775],
# [52.06635644064545, -0.6305292431981229],
# [52.06631677793901, -0.6308733422165709],
# [52.066306, -0.631039]]

# points=[]

# for i in coords:
#   lat=((i[0] + 90) / 180) * 11
#   lon=((i[1] + 180) / 360) * 11
#   points.append([lat,lon])

# print(points)
