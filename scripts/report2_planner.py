#!/usr/bin/env python3

import rospy
import math

# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
import rospy
import math
import numpy as np
import cv2
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import tf2_ros
from tf.transformations import *
import tf2_geometry_msgs

#initiate SphereParams
sphere_params = SphereParams()

#function to get sphere_params form SphereParams
def get_params(sphere_params1):
	global sphere_params
	sphere_params = sphere_params1

if __name__ == '__main__':
    # initialize the node
    rospy.init_node('simple_planner', anonymous = True)
    # add a subscriber to subscribe to sphere_params to get location of ball
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    plan_sub = rospy.Subscriber('/sphere_params', SphereParams, get_params)
    # add a publisher for sending joint position commands
    plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
    # set a 10Hz frequency for this loop
    loop_rate = rospy.Rate(10)
    # define a plan variable
    plan = Plan()
    

while not rospy.is_shutdown():
	try:
		trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		print("Frames Not Available!")
		loop_rate.sleep()
		continue
	pt_in_tool = tf2_geometry_msgs.PointStamped()
	pt_in_tool.header.frame_id = 'camera_color_optical_frame'
	pt_in_tool.header.stamp = rospy.get_rostime()
	
	pt_in_tool.point.x = sphere_params.xc
	pt_in_tool.point.y = sphere_params.yc
	pt_in_tool.point.z = sphere_params.zc
	pt_in_base = tfBuffer.transform(pt_in_tool, 'base', rospy.Duration(1.0))
	x = pt_in_base.point.x
	y = pt_in_base.point.y
	z = pt_in_base.point.z
	radius = sphere_params.radius
	
	#Motion 1: Starting place
	plan_point1 = Twist()
	plan_point1.linear.x = x
	plan_point1.linear.y = y
	plan_point1.linear.z = z + 0.1
	plan_point1.angular.x = -2.98
	plan_point1.angular.y = -0.05
	plan_point1.angular.z = -0.33
	# add point 1 to the plan
	plan.points.append(plan_point1)
	
	#Motion 2: "Pick Up" Object
	plan_point2 = Twist()
	plan_point2.linear.x = x
	plan_point2.linear.y = y
	plan_point2.linear.z = z + radius
	plan_point2.angular.x = -2.98
	plan_point2.angular.y = -0.13
	plan_point2.angular.z = 1.93
	# add point 2 to the plan
	plan.points.append(plan_point2)
	
	#Motion 3: Starting place
	plan_point3 = Twist()
	plan_point3.linear.x = x
	plan_point3.linear.y = y
	plan_point3.linear.z = z + 0.1
	plan_point3.angular.x = -2.98
	plan_point3.angular.y = -0.05
	plan_point3.angular.z = -0.33
	# add point 3 to the plan
	plan.points.append(plan_point3)
	
	#Motion 4: Move Object Further
	plan_point4 = Twist()
	plan_point4.linear.x = -0.2
	plan_point4.linear.y = 0.57
	plan_point4.linear.z = 0.19
	plan_point4.angular.x = 2.88
	plan_point4.angular.y = -0.05
	plan_point4.angular.z = .16
	# add point 4 to the plan
	plan.points.append(plan_point4)
	
	#Motion 5: Lowering Object
	plan_point5 = Twist()
	plan_point5.linear.x = -0.05
	plan_point5.linear.y = 0.66
	plan_point5.linear.z = -0.05
	plan_point5.angular.x = 3.00
	plan_point5.angular.y = -0.05
	plan_point5.angular.z = -0.089
	# add tpoint 5 to the plan
	plan.points.append(plan_point5)
	
	# publish the plan
	plan_pub.publish(plan)
	# wait for 0.1 seconds until the next loop and repeat
	loop_rate.sleep()
