#!/usr/bin/env python3

import rospy
import math

# import the plan message from the ur5e_control. Plan also available in this package!
from ur5e_control.msg import Plan
# import Twist
from geometry_msgs.msg import Twist
# import rospy
import rospy
# import math
import math
# import numpy
import numpy as np
# import cv2
import cv2
# import XYZarray
from robot_vision_lectures.msg import XYZarray
# import SphereParams
from robot_vision_lectures.msg import SphereParams
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
# import tf2_ros
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
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # add a subscriber to subscribe to sphere_params to get location of ball
    plan_sub = rospy.Subscriber('/sphere_params', SphereParams, get_params)
    # add a publisher for sending joint position commands
    plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
    # set a 10Hz frequency for this loop
    loop_rate = rospy.Rate(10)
    # define a plan variable
    plan = Plan()
    

while not rospy.is_shutdown():
	# Hand-Eye Corrdination Try and Except to check Frames.
	# Code from Saeidi, Hamed. University of North Carolina at Wilmington. hsaeidi-uncw/ur5e_control (github.com)
	try:
		trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		print("Frames Not Available!")
		loop_rate.sleep()
		continue
		
	pt_in_tool = tf2_geometry_msgs.PointStamped()
	pt_in_tool.header.frame_id = 'camera_color_optical_frame'
	pt_in_tool.header.stamp = rospy.get_rostime()
	
	# Get the points from sphere_params
	pt_in_tool.point.x = sphere_params.xc
	pt_in_tool.point.y = sphere_params.yc
	pt_in_tool.point.z = sphere_params.zc
	# transform the pts to base frame
	pt_in_base = tfBuffer.transform(pt_in_tool, 'base', rospy.Duration(1.0))
	# change the variable of each point and radius
	x = pt_in_base.point.x
	y = pt_in_base.point.y
	z = pt_in_base.point.z
	radius = sphere_params.radius
	# End code from Saeidi, Hamed. University of North Carolina at Wilmington. hsaeidi-uncw/ur5e_control (github.com)
	
	#Motion 1: Starting place
	plan_point1 = Twist()
	plan_point1.linear.x = 0.13
	plan_point1.linear.y = 0.7
	plan_point1.linear.z = 0.255
	plan_point1.angular.x = 3.14
	plan_point1.angular.y = 0.00
	plan_point1.angular.z = 3.14
	# add point 1 to the plan
	plan.points.append(plan_point1)
	
	#Motion 2: "Pick Up" Object
	plan_point2 = Twist()
	plan_point2.linear.x = x
	plan_point2.linear.y = y
	plan_point2.linear.z = z + radius
	plan_point2.angular.x = 3.14
	plan_point2.angular.y = 0.00
	plan_point2.angular.z = 3.14
	# add point 2 to the plan
	plan.points.append(plan_point2)
	
	#Motion 3: Starting place
	plan_point3 = Twist()
	plan_point3.linear.x = 0.13
	plan_point3.linear.y = 0.7
	plan_point3.linear.z = 0.255
	plan_point3.angular.x = 3.14
	plan_point3.angular.y = 0.00
	plan_point3.angular.z = 3.14
	# add point 3 to the plan
	plan.points.append(plan_point3)
	
	#Motion 4: Move Object Further
	plan_point4 = Twist()
	plan_point4.linear.x = -0.2
	plan_point4.linear.y = 0.57
	plan_point4.linear.z = 0.19
	plan_point4.angular.x = 3.14
	plan_point4.angular.y = 0.00
	plan_point4.angular.z = 3.14
	# add point 4 to the plan
	plan.points.append(plan_point4)
	
	#Motion 5: Lowering Object
	plan_point5 = Twist()
	plan_point5.linear.x = -0.2
	plan_point5.linear.y = 0.57
	plan_point5.linear.z = 0.05
	plan_point5.angular.x = 3.14
	plan_point5.angular.y = 0.00
	plan_point5.angular.z = 3.14
	# add tpoint 5 to the plan
	plan.points.append(plan_point5)
	
	#Motion 6: Lowering Object
	plan_point6 = Twist()
	plan_point6.linear.x = -0.2
	plan_point6.linear.y = 0.57
	plan_point6.linear.z = 0.19
	plan_point6.angular.x = 3.14
	plan_point6.angular.y = 0.00
	plan_point6.angular.z = 3.14
	# add tpoint 5 to the plan
	plan.points.append(plan_point6)
	
	# publish the plan
	plan_pub.publish(plan)
	# wait for 0.1 seconds until the next loop and repeat
	loop_rate.sleep()
