#! /usr/bin/env python 

import sys
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from autonomous_mode.msg import TrajStamped
from autonomous_mode.msg import RoverPoseGlobalStamped
from autonomous_mode.msg import FloatStamped
import numpy as np
import scipy.io as sio
import math

def pass_ref(Z):
	traj_msg = TrajStamped()
	traj_msg_header = Header(stamp=rospy.Time.now(),frame_id='base_link')
	traj_msg.ref_traj = Z.flatten()
	traj_msg.ref_size = np.size(Z[:,0],0) 
	traj_msg.Ts = 0.05;
	traj_pub.publish(traj_msg)

if __name__ == '__main__':

    try:
        rospy.init_node('pass_ref_to_traj_to_low',anonymous=False)

	#   Publications to ros topics
	traj_pub = rospy.Publisher('traj_to_low_RTD',TrajStamped,queue_size=100)

	# Load mat file(s).
	everything_mat = sio.loadmat('/home/nvidia/RTD_ws/src/autonomous_mode/Mat_Files/cartographer_traj/turn_left1_linsys_and_traj.mat')

	Z = everything_mat['ref_traj']
	rospy.sleep(3)
	pass_ref(Z)

    except rospy.ROSInterruptException:
	pass
	




