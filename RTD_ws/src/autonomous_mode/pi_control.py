#! /usr/bin/env python

# ------------------------------------------------------------------------
# pi control on steering angle using psi
# ------------------------------------------------------------------------

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from autonomous_mode.msg import TrajStamped
from autonomous_mode.msg import RoverPoseGlobalStamped
from std_msgs.msg import Header
import numpy as np
import math



def compute_u():
    global integral
    #
    #   Perform the pi action at the current ref
    #
    u[0] = ref[4]
    if t*6+2 >= len(ref):
	integral = (ref[-4]-pose.psi)*Ts+integral
    	u[1] = K_p*(ref[-4]-pose.psi) + K_i*integral + ref[5]
    else:
    	integral = (ref[t*6+2]-pose.psi)*Ts+integral
    	u[1] = K_p*(ref[t*6+2]-pose.psi) + K_i*integral + ref[5]
    nav_publisher(u)

# -------------------------------------------------------------
# Publisher functions
# -------------------------------------------------------------

def nav_publisher(u):
    nav_msg = AckermannDriveStamped()
    nav_msg.header = Header(stamp=rospy.Time.now(), frame_id='base_link')
    nav_msg.drive.speed = u[0]
    nav_msg.drive.steering_angle = u[1]
    nav_pub.publish(nav_msg) 


# --------------------------------------------------------------
# Callback functions
# --------------------------------------------------------------

def callback_pose(pose_):
    global pose
    pose = pose_

def callback_traj(Z):
    global ref, Ts, t, integral
    ref = Z.ref_traj
    Ts = Z.Ts
    t = 0
    integral = 0
    

# --------------------------------------------------------------
# Main function
# --------------------------------------------------------------

if __name__ == '__main__':
    global u,queue_load,pose,ref,Ts,t,integral

    try:
	rospy.init_node('open_loop_control',anonymous=False)
        u = [0,0]

	#   Subscribe to ros topics
	traj_sub = rospy.Subscriber('traj_to_low_RTD',TrajStamped,callback_traj)
	rospy.wait_for_message('traj_to_low_RTD',TrajStamped)

	pose_sub = rospy.Subscriber('cart_rover_global',RoverPoseGlobalStamped,callback_pose)
	rospy.wait_for_message('cart_rover_global',RoverPoseGlobalStamped)

        #   Publish to nav_0 topic (give acutation commands)
        nav_pub = rospy.Publisher('vesc/high_level/ackermann_cmd_mux/input/nav_0', 
            AckermannDriveStamped, queue_size=10000)

	K_p = 0.01
	K_i = 0.01

        while not rospy.is_shutdown():
	    compute_u()
	    rospy.sleep(Ts)
	    t = t+1
	rospy.spin()

    except rospy.ROSInterruptException:
        pass
    
    
