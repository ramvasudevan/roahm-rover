#! /usr/bin/env python

#--------------------------------------------------------------------------
# This script gives actuation commands for the rover, based on
# a linearized state feedback control with Luenberger observer,
# with time parameterized matrices, passed in a TrajControlStamped message.
#--------------------------------------------------------------------------

import rospy
from ackermann_msgs.msg import AckermannDriveStamped 
from autonomous_mode.msg import RoverPoseGlobalStamped
from std_msgs.msg import Header
from autonomous_mode.msg import TrajControlStamped
import numpy as np
import scipy.io as sio



def scheduler():
    global traj_flag,H,y_first,drive_size

    traj_flag = 0
    drive_end = 0
    drive_size = 0

    while not rospy.is_shutdown():
        if not drive_end:
            for ind in range(drive_size): 
                if traj_flag:
                    break
                if ind == drive_size-1:
                    drive_end = 1
                drive(ind)
                rate.sleep()
        if traj_flag:
            rate = rospy.Rate(1/Ts)
            traj_flag = 0
            drive_end = 0
            pose_first = rospy.wait_for_message('cart_rover_global', RoverPoseGlobalStamped)
            y_first[:,:] = [[pose_first.x],[pose_first.y],[pose_first.psi],[0]]
            H = calc_transform(y_first)


def callback_traj(data):
    global A,B,C,D,K,L,Z,drive_size,Ts,traj_flag

    drive_size = int(data.ref_points)
    A = np.reshape(data.A,(4,4,drive_size))
    B = np.reshape(data.B,(4,2,drive_size))
    C = np.reshape(data.C,(4,4,drive_size))
    K = np.reshape(data.K,(2,4,drive_size))
    L = np.reshape(data.L,(4,4,drive_size))
    Z = np.reshape(data.Z,(drive_size,6))
    Ts = data.Ts
    traj_flag = 1

def callback_pose(data):
    global y_raw

    y_raw[:,:] = [[data.x],[data.y],[data.psi],[0]]

def drive(ind):
    global y,y_raw,u,ex_hat

    #   Get pose of rover
    y = calc_y(y_raw, H, y_first)

    #   Calculate control input
    u = calc_u(K[:,:,ind], ex_hat, np.expand_dims(Z[ind,4:6],axis=1))
    nav_publisher(u)

    #   Propogate the observer model
    ex_hat = calc_ex_hat(A[:,:,ind], B[:,:,ind], C[:,:,ind], K[:,:,ind], L[:,:,ind], 
	ex_hat, y, np.expand_dims(Z[ind,0:4],axis=1))

def calc_transform(y_input):
    H = np.array([[np.cos(y_input[2]),-np.sin(y_input[2]),y_input[0]],
        [np.sin(y_input[2]),np.cos(y_input[2]),y_input[1]],[0,0,1]],dtype='float')
    return H

def calc_y(y_raw,H,y_first):
    y_inter = np.matmul(np.linalg.inv(H),np.append(y_raw[0:2],[[1]])).reshape((-1,1))
    y = np.append(y_inter[0:2],[y_raw[2]-y_first[2]],axis=0)
    y = np.append(y,[y_raw[3]-y_first[3]],axis=0)
    return y


def calc_u(K, ex_hat, u_ref):
    u = -np.matmul(K,ex_hat) + u_ref
    return u

def calc_ex_hat(A, B, C, K, L, ex_hat, y, x_ref):
    y_hat = np.matmul(C,ex_hat)
    ex_hat_next = np.matmul((A-np.matmul(B,K)),ex_hat)-np.matmul(L,y_hat-np.matmul(C,y-x_ref))
    return ex_hat_next

def nav_publisher(u):
    nav_msg = AckermannDriveStamped()
    nav_msg.header = Header(stamp=rospy.Time.now(), frame_id='base_link')
    nav_msg.drive.speed = u[0]
    nav_msg.drive.steering_angle = u[1]
    nav_pub.publish(nav_msg)

if __name__ == '__main__':
    global u,y,y_raw,y_first,H,ex_hat
    try:
        rospy.init_node('state_feedback_online_drive', anonymous=False)
        u = np.zeros((2,1))
        y = np.zeros((4,1))
	y_raw = np.zeros((4,1))
        y_first = np.zeros((4,1))
        H = np.zeros((3,3))
        ex_hat = np.zeros((4,1))

        #   Subscribe to traj_drive topic
        traj_sub = rospy.Subscriber('traj_drive', TrajControlStamped, callback_traj)

	#   Subscribe to mocap_rover_global topic
	pose_sub = rospy.Subscriber('cart_rover_global', RoverPoseGlobalStamped, callback_pose)

        #   Publish to nav_0 topic (give actuation commands)
        nav_pub = rospy.Publisher('vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped,
            queue_size=100)

        #   Run sequencer
        scheduler()

	rospy.spin()

    except rospy.ROSInterruptException:
        pass
