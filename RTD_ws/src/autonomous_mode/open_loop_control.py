#! /usr/bin/env python

# ------------------------------------------------------------------------
# Open loop control- simply give the rover the current commands.
# ------------------------------------------------------------------------

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from autonomous_mode.msg import TrajStamped
from std_msgs.msg import Header
import numpy as np
import math

# -------------------------------------------------------------
# Publisher and Subscriber functions
# -------------------------------------------------------------

def callback_traj(Z):
    global u
    u = [0,0]
    u[0] = Z.ref_traj[4]
    u[1] = Z.ref_traj[5]
    
    # Decrease deadzone 
    if abs(u[0]) < 0.5:
	u[0] = u[0]+0.2

    # decrease steering bias
    #if abs(u[1]) < 0.1:
    #    u[1] = u[1]-0.05
	

def nav_publisher(u):
    nav_msg = AckermannDriveStamped()
    nav_msg.header = Header(stamp=rospy.Time.now(), frame_id='base_link')
    nav_msg.drive.speed = u[0]
    nav_msg.drive.steering_angle = u[1]
    nav_pub.publish(nav_msg) 


# --------------------------------------------------------------
# Main function
# --------------------------------------------------------------

if __name__ == '__main__':
    global u,queue_load

    try:
	rospy.init_node('open_loop_control',anonymous=False)
        u = [0,0]

	#   Subscribe to traj_to_low_RTD topic
	traj_sub = rospy.Subscriber('traj_to_low_RTD',TrajStamped,callback_traj)

        #   Publish to nav_0 topic (give acutation commands)
        nav_pub = rospy.Publisher('vesc/high_level/ackermann_cmd_mux/input/nav_0', 
            AckermannDriveStamped, queue_size=10000)

        while not rospy.is_shutdown():
	    nav_publisher(u)

	rospy.spin()

    except rospy.ROSInterruptException:
        pass
    
    
