#! /usr/bin/env python

#---------------------------------------------------------------------
# This script offers one low-level controller implementation of 
# the RTD technique, an LQR controller, and publishes a 
# TrajControlStamped message.
#
# Search ADJUSTABLE to find easily changeable parameters
#---------------------------------------------------------------------

import rospy
from std_msgs.msg import Header
from autonomous_mode.msg import TrajControlStamped
from autonomous_mode.msg import TrajStamped
import numpy as np
import scipy.io as sio
import scipy.signal as ssig


def callback_traj(Z):
    global Ts

    ref_size = int(Z.ref_size)
    ref_traj = np.reshape(Z.ref_traj,(ref_size,6))
    Ts = Z.Ts

    preprocess_for_update(ref_size,ref_traj)
    for ind in range(ref_size):
        linearize(ind)
        discretize(ind)
    lqr(ref_size)
    traj_publisher(ref_size)


def initialize_data(ref_size):
    global A,B,C,D,A_d,B_d,C_d,D_d,K_d,L_d,Q_K,R_K,Q_L,R_L,S_K,S_L,A_perm,C_perm,L_perm,Z
   
    #   Continuous Time 
    A = np.zeros((4,4,ref_size),dtype=float)
    B = np.zeros((4,2,ref_size),dtype=float)
    C = np.zeros((4,4,ref_size),dtype=float)
    D = np.zeros((4,2,ref_size),dtype=float)

    for ind in range(ref_size):
        C[:,:,ind] = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,0]]

    #   Discrete Time
    A_d = np.zeros((4,4,ref_size),dtype=float)
    B_d = np.zeros((4,2,ref_size),dtype=float)
    C_d = np.zeros((4,4,ref_size),dtype=float)
    D_d = np.zeros((4,2,ref_size),dtype=float)
    K_d = np.zeros((2,4,ref_size),dtype=float)
    L_d = np.zeros((4,4,ref_size),dtype=float)

    #   LQR tuning ADJUSTABLE
    Q_K = np.diag([50,50,50,30])
    R_K = np.diag([20,10])
    Q_L = np.diag([1,1,1,1])
    R_L = np.diag([1,1,1,1])

    #   LQR support matrices
    S_K = np.zeros((4,4,ref_size),dtype=float)
    S_L = np.zeros((4,4,ref_size),dtype=float)
    A_perm = np.zeros((4,4,ref_size),dtype=float)
    C_perm = np.zeros((4,4,ref_size),dtype=float)
    L_perm = np.zeros((4,4,ref_size),dtype=float)

    #   Initialize ref_traj
    Z = np.zeros((ref_size,6))


def preprocess_for_update(ref_size,ref_traj):
    global A,B,A_d,B_d,C_d,D_d,K_d,L_d,S_K,S_L,A_perm,C_perm,L_perm,Z

    #   Check initialization large enough
    ref_size_lim = np.size(A,2)
    if ref_size_lim < ref_size:
	ref_size_init = np.floor(ref_size*1.5)
	initialize_data(ref_size_init.astype(int))

    #   Reassign Z and zero out trajectories
    Z = Z*0
    Z[range(ref_size),:] = ref_traj
    A = A*0
    B = B*0
    A_d = A_d*0
    B_d = B_d*0
    C_d = C_d*0
    D_d = D_d*0
    K_d = K_d*0
    L_d = L_d*0
    S_K = S_K*0
    S_L = S_L*0
    A_perm = A_perm*0
    C_perm = C_perm*0
    L_perm = L_perm*0


def linearize(ind):
    global A,B

    A[0,2,ind] = (-Z[ind,3]*np.sin(Z[ind,2])-(np.tan(c[0]*Z[ind,5]+c[1])*Z[ind,3]/
        (c[2]+c[3]*Z[ind,3]**2))*(c[7]+c[8]*Z[ind,3]**2)*np.cos(Z[ind,2]))

    A[0,3,ind] = (np.cos(Z[ind,2])-np.tan(c[0]*Z[ind,5]+c[1])*(c[7]*np.sin(Z[ind,2]))*
        ((c[2]-c[3]*Z[ind,3]**2)/((c[2]+c[3]*Z[ind,3]**2)**2))-np.tan(c[0]*Z[ind,5]+c[1])*(c[8]
        *np.cos(Z[ind,2]))*((3*c[2]*Z[ind,3]**2+c[3]*Z[ind,3]**4)/((c[2]+c[3]*Z[ind,3]**2)**2)))

    A[1,2,ind] = (Z[ind,3]*np.cos(Z[ind,2])-(np.tan(c[0]*Z[ind,5]+c[1])*Z[ind,3]/
        (c[2]+c[3]*Z[ind,3]**2))*(c[7]+c[8]*Z[ind,3]**2)*np.sin(Z[ind,2]))

    A[1,3,ind] = (np.sin(Z[ind,2])+np.tan(c[0]*Z[ind,5]+c[1])*(c[7]*np.cos(Z[ind,2]))*
        ((c[2]-c[3]*Z[ind,3]**2)/((c[2]+c[3]*Z[ind,3]**2)**2))+np.tan(c[0]*Z[ind,5]+c[1])*
        (c[8]*np.cos(Z[ind,2]))*((3*c[2]*Z[ind,3]**2+c[3]*Z[ind,3]**4)/((c[2]+c[3]*Z[ind,3]**2)**2)))

    A[2,3,ind] = np.tan(c[0]*Z[ind,5]+c[1])*(c[2]-c[3]*(Z[ind,3]**2))/((c[2]+c[3]*(Z[ind,3]**2))**2)

    A[3,3,ind] = c[5]+2*c[6]*(Z[ind,3]-Z[ind,4])

    B[0,1,ind] = ((-c[0]*Z[ind,3]*(1/(np.cos(c[0]*Z[ind,5]+c[1])**2))/
        (c[2]+c[3]*(Z[ind,3]**2)))*(c[7]+c[8]*Z[ind,3]**2)*np.sin(Z[ind,2]))

    B[1,1,ind] = ((c[0]*Z[ind,3]*(1/(np.cos(c[0]*Z[ind,5]+c[1])**2))/
        (c[2]+c[3]*(Z[ind,3]**2)))*(c[7]+c[8]*Z[ind,3]**2)*np.cos(Z[ind,2]))

    B[2,1,ind] = c[0]*Z[ind,3]*(1/(np.cos(c[0]*Z[ind,5]+c[1])**2))/(c[2]+c[3]*(Z[ind,3]**2))

    B[3,0,ind] = -c[5]-2*c[6]*(Z[ind,3]-Z[ind,4])


def discretize(ind):
    global A_d,B_d,C_d,D_d
    
    sysd = ssig.cont2discrete((A[:,:,ind],B[:,:,ind],C[:,:,ind],D[:,:,ind]),Ts,method="zoh")
    A_d[:,:,ind] = sysd[0]  
    B_d[:,:,ind] = sysd[1]  
    C_d[:,:,ind] = sysd[2]
    D_d[:,:,ind] = sysd[3]


def lqr(ref_size):
    global K_d,L_d,S_K,S_L,A_perm,C_perm,L_perm
	
    A_perm[:,:,:] = np.transpose(A_d,(1,0,2))
    C_perm[:,:,:] = np.transpose(C_d,(1,0,2))

    S_K[:,:,ref_size-1] = Q_K;
    S_L[:,:,ref_size-1] = Q_L;
    K_d[:,:,ref_size-1] = [[0,0,0,0],[0,0,0,0]]
    L_d[:,:,ref_size-1] = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]

    for indf in range(ref_size-1):

        ind = ref_size-2-indf

        S_K[:,:,ind] = (np.matmul(np.matmul(np.transpose(A_d[:,:,ind]),S_K[:,:,ind+1]),A_d[:,:,ind])
            -np.matmul(np.matmul(np.matmul(np.matmul(np.transpose(A_d[:,:,ind]),S_K[:,:,ind+1]),
            B_d[:,:,ind]),np.linalg.inv(R_K+np.matmul(np.matmul(np.transpose(B_d[:,:,ind]),
            S_K[:,:,ind+1]),B_d[:,:,ind]))),np.matmul(np.matmul(np.transpose(B_d[:,:,ind]),
	    S_K[:,:,ind+1]),A_d[:,:,ind]))+Q_K)

        K_d[:,:,ind] = (np.matmul(np.linalg.inv(R_K+np.matmul(np.matmul(np.transpose(B_d[:,:,ind]),
            S_K[:,:,ind+1]),B_d[:,:,ind])),np.matmul(np.matmul(np.transpose(B_d[:,:,ind]),
	    S_K[:,:,ind+1]),A_d[:,:,ind])))
    
        S_L[:,:,ind] = (np.matmul(np.matmul(np.transpose(A_perm[:,:,ind]),S_L[:,:,ind+1]),A_perm[:,:,ind])
            -np.matmul(np.matmul(np.matmul(np.matmul(np.transpose(A_perm[:,:,ind]),S_L[:,:,ind+1]),
            C_perm[:,:,ind]),np.linalg.inv(R_L+np.matmul(np.matmul(np.transpose(C_perm[:,:,ind]),
            S_L[:,:,ind+1]),C_perm[:,:,ind]))),np.matmul(np.matmul(np.transpose(C_perm[:,:,ind]),
	    S_L[:,:,ind+1]),A_perm[:,:,ind]))+Q_L)
   
        L_perm[:,:,ind] = (np.matmul(np.linalg.inv(R_L+np.matmul(np.matmul(np.transpose(C_perm[:,:,ind]),
            S_L[:,:,ind+1]),C_perm[:,:,ind])),np.matmul(np.matmul(np.transpose(C_perm[:,:,ind]),
	    S_L[:,:,ind+1]),A_perm[:,:,ind])))

    L_d = np.transpose(L_perm,(1,0,2))


def traj_publisher(ref_size):
    traj_msg = TrajControlStamped()
    traj_msg.header = Header(stamp=rospy.Time.now(),frame_id='base_link')
    traj_msg.A = A_d[:,:,range(ref_size)].flatten()
    traj_msg.B = B_d[:,:,range(ref_size)].flatten()
    traj_msg.C = C_d[:,:,range(ref_size)].flatten()
    traj_msg.K = K_d[:,:,range(ref_size)].flatten()
    traj_msg.L = L_d[:,:,range(ref_size)].flatten()
    traj_msg.Z = Z[range(ref_size),:].flatten()
    traj_msg.ref_points = ref_size
    traj_msg.Ts = Ts
    traj_pub.publish(traj_msg)


if __name__ == '__main__':
    try:
	rospy.init_node('lqr_control', anonymous=False)

        #   Subscribe to traj_to_low_RTD topic
        traj_sub = rospy.Subscriber('traj_to_low_RTD',TrajStamped, callback_traj)

        #   Publish to traj_Drive topic
        traj_pub = rospy.Publisher('traj_drive',TrajControlStamped, queue_size=100)

        #   Initialize Data
        ref_size_init = 500 #ADJUSTABLE
        initialize_data(ref_size_init)

        #   Load coefficients #ADJUSTABLE
        c_mat = sio.loadmat('/home/nvidia/RTD_ws/src/autonomous_mode/Mat_Files/newest_traj_lqr.mat')
        c = c_mat['coeffs']
        c = np.squeeze(c,0)
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


