#! /usr/bin/env python

# -----------------------------------------------------------------------------
# This script implements the online component of RTD.
# It intersects the offline generated FRS (Forward Reachable Set) with 
# obstacles online and optimizes for a trajectory based on the safe parameters.
# 
# Adopted from matlab simulation code found at 
# https://github.com/skousik/RTD_tutorial
# https://github.com/ramvasudevan/RTD
#
# Search ADJUSTABLE to find easily changed parameters
#
# Author of this script: Steven van Leeuwen (svanlee@umich.edu)
# -----------------------------------------------------------------------------

import sys
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from autonomous_mode.msg import TrajStamped
from autonomous_mode.msg import RoverPoseGlobalStamped
from autonomous_mode.msg import FloatStamped
from rover_dynamics import rover_dynamics
from FRS_solver_obj import solver_obj
import ipopt
import numpy as np
import scipy.io as sio
import math

#------------------------------------------------------------------------------
#   These functions are support functions for the scipt, do not relate to 
#   the RTD pipeline
#------------------------------------------------------------------------------

def MSG_TIME(msg1=None,msg2=None,msg3=None,msg4=None):
    #
    #   Print to std out is msg1 given
    #
    global time_start

    if (msg1 != True):
        return
    if time_start == 0:
        time_start = rospy.get_time()
	return
    else:
        print msg2,
        time_rec = rospy.get_time()
        print '-Elapsed time:',time_rec-time_start
    if type(msg3) is int:
        print '            Number of constraints:',msg3
    elif type(msg3) is tuple:
	print '           ',['%.3f' % k for k in msg3]
	print '           ',['%.3f' % k for k in msg4]
    elif type(msg3) is np.ndarray:
        print '            Control commands:',msg3
        print '- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -'
	print '                                                                       '
	print '- - - - - - - - - - - - - - B E G I N - - - - - - - - - - - - - - - - -'
        time_start = 0

#------------------------------------------------------------------------------
#   These functions have to do with processing the obstacle points.
#------------------------------------------------------------------------------

def gen_buf_discrete(oc_box_xy,oc_wh,oc_1,data,oc_buffer,oc,xy0,res):
    #
    #   Generate occupancy points a certain spacing around each occupied point
    #   When the buffer generation is done, add the buffer points as constraints
    #   to the solver format.
    #
    obs = []

    #   Buffer each point registered with oc_buffer.
    for x in oc_box_xy[0]:
        for y in oc_box_xy[1]:
            if data[y*oc_wh[0]+x] >= oc_1:
		oc[x-oc_buffer:x+oc_buffer+1,y-oc_buffer:y+oc_buffer+1] = 1

    #   Remove interior points and add to obstacle list.
    for x in oc_box_xy[0]:
        for y in oc_box_xy[1]:
            if oc[x,y] == 1 and not sum_surround_oc(x,y,oc,oc_wh) == 8:
                #   Put obstacles in FRS frame. Shift to initial_x, initial_y
		#   as in FRS computation. 
                obs_FRS = obs_map_2_FRS(res*x+xy0[0],res*y+xy0[1],pose_future[0:3])

                #   Append as a constraint if it is reachable in the FRS frame.
		#   Further trim the [-1,1] box according to FRS countour to reduce 
		#   number of constraints.
                if (obs_FRS[0] >= box_FRS[0] and obs_FRS[0] <= box_FRS[1]
		and obs_FRS[1] >= box_FRS[2] and obs_FRS[1] <= box_FRS[3]):
                    obs.append(obs_FRS) 
    return obs


def check_buf_discrete_ok(res,d,W):
    #
    #   It is possible the rover can pass far enough through the discretized buffer.
    #   Ensure this is not the case.
    #
    a = res/2     
    if (W <= res):
        return 0
    buf = res+(d-1)*res 
    if (buf <= a):
        return 0
    return 1


def gen_oc_box_ind(ind,ind_expand,lim,oc_buffer):
    #
    #   Make the list of indicies of the occupancy grid for 
    #   occupancy consideration.
    #
    oc_box = np.arange(max(ind-((ind_expand-1)/2),oc_buffer),
	min(ind+((ind_expand-1)/2),lim-oc_buffer))
    return oc_box


def sum_surround_oc(x,y,oc,oc_wh):
    #
    #   A utility function to gather whether a point is an interior point.
    #
    sum_ = 0
    if not (x < 1 or x >= oc_wh[0]-1):
        if not (y < 1 or y >= oc_wh[1]-1):
            sum_ = (oc[x-1,y-1]+oc[x-1,y]+oc[x-1,y]+oc[x,y-1]+
                oc[x,y+1]+oc[x+1,y-1]+oc[x+1,y]+oc[x+1,y+1])
    return sum_


#------------------------------------------------------------------------------
#   These functions have to do with transforming between the occupancy map
#   and FRS frames.
#------------------------------------------------------------------------------

def obs_map_2_FRS(x,y,pose):
    #
    #   The FRS frame has the rover heading as zero heading, and in this 
    #   implementation at (0,0) position.
    #
    pose_xy = np.array([[x-pose[0]],[y-pose[1]]])
    R = np.array([[np.cos(pose[2]),np.sin(pose[2])],
        [-np.sin(pose[2]),np.cos(pose[2])]])
    pose_frs = (1/dist_scale)*np.matmul(R,pose_xy)
    return [pose_frs[0]+initial_x,-pose_frs[1]+initial_y]


def k_FRS_2_map(k):
    #
    #   Scale the k parameters to their control commands.
    #
    U = [0,0]
    U[0] = k[0]*k_scale[0]+k_const
    U[1] = k[1]*k_scale[1]
    return U


def cart_rover_global_2_map():
    #
    #   Get pose into map frame. Additionally 
    #   get velocity estimate.
    #
    return (pose_raw.x,-pose_raw.y,-pose_raw.psi,vel_est)
	

#------------------------------------------------------------------------------
#   These functions have to do with constraint generation, setting up, and 
#   solving for the optimization of k.
#------------------------------------------------------------------------------

def proj_obs_to_k(obs):
    #
    #   For each obstacle point, a constraint polynomial as a function of 
    #   k is generated
    #
    n = len(obs)
    z_0 = np.zeros((n,2))

    # Create solver_obj object that is in the ipopt solver form.
    constr_obj = solver_obj(rover_dynamics,oc_box_xy,w_p,w_c,w_k)

    for k in range(n):
        z_0[k,:] = [round(obs[k][0],2),round(obs[k][1],2)]
        p_tmp = w_p.astype(float)

        #   Evaluate all powers of the current z variable in w
        c_tmp = w_c.astype(float)*np.power(z_0[k,0],w_p[:,w_z[0]])*\
	    np.power(z_0[k,1],w_p[:,w_z[1]])

	#   Zero out z columns
        p_tmp[:,w_z[0]] = 0
	p_tmp[:,w_z[1]] = 0

        #   Realize the obstacle point as a constraint.
        constr_obj.add_constraint(p_tmp,c_tmp,w_k)

    constr_obj.finalize()
    return constr_obj


def solver_opt_k_FRS(solver_obj):
    #
    #   Calls the solver to optimize over the cost function given constraints.
    #
    global brake

    #   Solver options
    k_lb = [-1,-1]
    k_ub = [1,1]
    c_lb = -1e19*np.ones(solver_obj.cnt)
    c_ub = np.ones(solver_obj.cnt)

    solver_obj.set_goal(Z_FRS_goal)
    nlp = ipopt.problem(len(opt_result),len(c_ub),problem_obj=solver_obj,
            lb=k_lb,ub=k_ub,cl=c_lb,cu=c_ub)

    #   Below are ADJUSTABLE*********************************************
    t_opt = 0.07          #   Min time alloted to do optimization. If less than this time 
                          #   before t_plan will not do optimization.
    t_latency = 0.12      #   Unusable time at the end of the planning period to 
			  #   account for latency. This time is subtracted from 
			  #   the computed optimization time allowed.
    nlp.addOption('tol',0.1)
    nlp.addOption('acceptable_iter',4)
    nlp.addOption('acceptable_tol',0.1)
    nlp.addOption('linear_solver','ma97')
                          #   Options specific to Ipopt.
                          #   See online documentation 
                          #   for a description of all options
    if solver_print_:
        nlp.addOption('print_level',3)
    else:
	nlp.addOption('print_level',0)
    #   End ADJUSTABLE***************************************************
    
    #   Decide if the algorithm should undergo optimization.
    t_j_to_now = rospy.get_time()-t_j

    if t_j_to_now+t_opt > t_plan:
	brake = True
    else:
        #   Call solver	  
	#   Initial point likely to be feasible
	nlp.addOption('max_cpu_time',max(0.02,round(t_plan-t_j_to_now-t_latency,2))) 
        (result,result_info) = nlp.solve([-1,0]) 

	#   Check status since latency can cause time beyond t_opt
	#   Allow suboptimal solution that satisifies constraint violation
	if rospy.get_time()-t_j > t_plan:
	    brake = True
	elif (result_info['status'] == 0 or result_info['status'] == 1):
	    brake = False
	elif result_info['status'] == -4 and solver_obj.constr_viol < 1e-3:
	    brake = False
        else:
	    brake = True

    if brake == True:
	result = [-float(k_const)/k_scale[0],opt_result[1]]
	MSG_TIME(print_,'- -\n- - **BRAKING**\n- - **Optimization not used**')

    return result


#------------------------------------------------------------------------------
#   This function is executed every time a new k is desired. All above 
#   functions are called through the algorithm_main.
#------------------------------------------------------------------------------

def algorithm_main(oc):
    #
    #   This function processes the occupancy grid from the environment
    #   and produces a trajectory to track.
    #
    global oc_box_xy,tpoints_traj_push,pose_future,ts_h,Z_FRS_goal,Z_h_roverinit

    MSG_TIME(print_)

    #   Get pose in map frame, the same frame the occupancy grid is in
    pose_init = cart_rover_global_2_map()

    #   Parse Occupancy Grid
    oc_wh = (oc.info.width,oc.info.height)
    oc_res = oc.info.resolution
    oc_xy0 = (oc.info.origin.position.x,oc.info.origin.position.y)
    oc_ = np.zeros((oc_wh[0],oc_wh[1]),dtype=int)
    oc_rover_xy_ind = []
    oc_box_xy = []

    #   Below are ADJUSTABLE*********************************************
    oc_buffer = 1           #   Distance in grid points for the buffer. 
                            #   For example oc_buffer=1 registers the 8 surrounding 
                            #   grid points as occupied.
    oc_box_size = 4.0       #   Centered box area side length, (m), to register occupied 
		            #   grid points
    oc_1 = 60               #   Confidence level criteria of oc grid point 0(none)-100(max)
    ts_h = 0.05             #   Time step for ref traj generation and future pose
                 	    #   calculation with high fid dynamics
    #   End ADJUSTABLE***************************************************
    
    #   Estimate future pose of rover and form box to evaluate obstacles
    #   Calculate future pose at t_plan elapsed time
    displ = rover_dynamics.flowf('low','no_record',[0,0,0,pose_init[3]],
        [0,t_plan],ts_h)
    pose_future = np.array([pose_init[0],pose_init[1],0,0])+\
	rover_dynamics.roverinit_2_map(displ,pose_init[2])
    MSG_TIME(print_,'- - **Poses calculated, current pose and xy goal**',
	pose_init,Z_map_goal)

    #   Only looking at the occupancy grid in a conservative box area around
    #   the pose of rover will further reduce the computation time.
    oc_box_ind_expand = int(math.floor((oc_box_size/oc_res)))
    if oc_box_ind_expand%2 == 0:
        oc_box_ind_expand = oc_box_ind_expand+1

    for k in range(2):
        oc_rover_xy_ind.append(int(math.floor((pose_future[k]-oc_xy0[k])/oc_res)))
        oc_box_xy.append(gen_oc_box_ind(oc_rover_xy_ind[k],oc_box_ind_expand,
	    oc_wh[k],oc_buffer))

    #   Buffer obstacles and obtain objects which to evaulate FRS against
    #   Rover is estimated to have a rectangular footprint, L is longitudinal, 
    #   W is lateral (m)
    if not check_buf_discrete_ok(oc_res,oc_buffer,footprint_W):
	MSG_TIME(print_,'- - **Discretization of buffer not ok**')

    obs = gen_buf_discrete(oc_box_xy,oc_wh,oc_1,oc.data,oc_buffer,
	oc_,oc_xy0,oc_res)
    MSG_TIME(print_,'- - **Generated buffer**')

    #   Find the unsafe k parameters (constraint polynomials in k)
    solver_obj = proj_obs_to_k(obs)
    MSG_TIME(print_,'- - **Generated constraints**',solver_obj.cnt)

    #   Decision to optimize and if yes then do 
    Z_FRS_goal = obs_map_2_FRS(Z_map_goal[0],Z_map_goal[1],pose_future)
    opt_result = solver_opt_k_FRS(solver_obj)
    MSG_TIME(print_,'- - **Optimized**')

    #   Realize the traj with chosen k
    opt_U = k_FRS_2_map(opt_result) 
    rover_dynamics.pass_U(opt_U)
    if brake:
        t_horz = t_stop-(t_stop%ts_h)+ts_h
    else:
        t_horz = t_plan-(t_plan%ts_h)+ts_h

    tpoints_traj_push = np.linspace(0,t_horz,num=math.ceil(1+t_horz/ts_h))
    X_h_roverinit = rover_dynamics.flowf('high','record',[0,0,0,pose_future[3]],
        tpoints_traj_push,ts_h)

    #   Convention in use is X is [x,y,psi,vel], while Z is [X' U']' 
    Z_h_roverinit = np.concatenate((X_h_roverinit,
        opt_U*np.ones((np.size(X_h_roverinit,0),2))),axis=1)
    
    #   Push traj_to_low_RTD
    while rospy.get_time()-t_j < t_plan and not brake:
        #   Wait until t_plan elapsed time to push
	pass

    traj_publisher(Z_h_roverinit) 
    MSG_TIME(print_,'- - **Done**',Z_h_roverinit[0,4:])


#------------------------------------------------------------------------------
#   Publisher functions
#------------------------------------------------------------------------------

def traj_publisher(Z):
    #
    #   Format the message that contains the trajectory to track, and publish. 
    #
    global t_j

    traj_msg = TrajStamped()
    traj_msg_header = Header(stamp=rospy.Time.now(),frame_id='base_link')
    traj_msg.ref_traj = Z.flatten()
    traj_msg.ref_size = np.size(Z[:,0],0) 
    traj_msg.Ts = ts_h;
    traj_pub.publish(traj_msg)
    while rospy.get_time()-t_j < t_plan and brake:
        #   Wait until t_plan to report t_j
	pass
    t_j = rospy.get_time()


#------------------------------------------------------------------------------
#   Callback functions
#------------------------------------------------------------------------------

def callback_oc(occupancy_grid):
    #
    #   This function gathers the occupancy grid from the environment
    #
    global cur_oc

    cur_oc = occupancy_grid

def callback_pose(pose):
    #
    #   This function gathers the pose from the environment
    #
    global pose_raw

    pose_raw = pose

def callback_goal(goal):
    #
    #   This function gets the goal from the planner
    #
    global Z_map_goal

    Z_map_goal = goal.data

def callback_odom(odom):
    #    
    #   Calculate vx 
    #
    global vel_est

    vel_est = abs(np.cos(rover_dynamics.U[1]))*odom.twist.twist.linear.x

#------------------------------------------------------------------------------
#   Main
#------------------------------------------------------------------------------

if __name__ == '__main__':
    #
    #   Default main function. Will enter into algorithm_main.
    #
    global w_p,w_c,w_z,w_k,dist_scale,k_scale,k_const,t_plan,t_stop,footprint_W,\
        initial_x,initial_y,rover_dynamics,Z_h_roverinit,pose_raw,pose_rate,\
	Z_map_goal,time_start,t_j,print_,solver_print_,opt_result,brake,box_FRS

    try:
        rospy.init_node('FRS_intersect_opt',anonymous=False)

	#   Initialize
	time_start = 0
        tpoints_traj_push = 0
	Z_h_roverinit = np.zeros((1,6))
	cur_oc = None
	pose_rate = 15.0
	opt_result = [0,0]

        #   Load FRS data structure
        #   -Assume not sparse
        #   -Assume w_z and w_k are each size 2, with the lower indexed column x,
        #    higher indexed column y for w_z
        frs = sio.loadmat(
            '/home/nvidia/RTD_ws/src/autonomous_mode/Mat_Files/FRS/rover_FRS_deg_10.mat')
        w_p = frs['pows'] 
        w_c = np.squeeze(frs['coef']) 
        w_z = np.squeeze(frs['z_cols'])
        w_k = np.squeeze(frs['k_cols'])
        dist_scale = np.squeeze(frs['dist_scale'])
	k_scale = np.squeeze(frs['k_scale'])
        k_const = np.squeeze(frs['k_const'])
        t_f = np.squeeze(frs['t_f'])
        t_plan = np.squeeze(frs['t_plan'])
        t_stop = np.squeeze(frs['t_stop'])
        footprint_W = np.squeeze(frs['footprint_W'])
        wb = np.squeeze(frs['wb'])
	initial_x = np.squeeze(frs['initial_x'])
	initial_y = np.squeeze(frs['initial_y'])
	box_FRS = np.squeeze(frs['box_FRS'])

	coeffile = sio.loadmat(
            '/home/nvidia/RTD_ws/src/autonomous_mode/Mat_Files/newest_traj_lqr.mat')
	c = coeffile['coeffs']
	c = np.squeeze(c,0)

	#   Below are ADJUSTABLE**************************************************
	Z_map_goal = [-4.04,4.633]
                              #   Goal position for the rover for single goal mode, 
                              #   At where it would not move any further.
        print_ = True         #   Print elapsed time and diagnostic information to 
                              #   to standard out.
	solver_print_ = True  #   Print solver specific information.
        #   End ADJUSTABLE********************************************************
        
        #   Check for high-level planner
        try:
            rospy.wait_for_message('goal_to_mid_RTD',FloatStamped,timeout=0.2)
            planner = True
	    print('Starting RTD with planner')
        except:
	    #   Single goal mode
            #   Check for Z_map_goal entered on command line
	    y_n = raw_input('Starting RTD. No planner. Enter Z_map_goal (y/n): ')
	    try: 
	        if y_n == 'y':
	            gx,gy = raw_input().split()
	            Z_map_goal = [float(gx),float(gy)]
	    except:
	        pass
            planner = False

	#   Create rover dynamics object
	#   NOTE the lambda functions as found in the rover_dynamics class
        #   are FRS specific for fast runtime
        rover_dynamics = rover_dynamics()
	rover_dynamics.pass_coeff(c,wb,(t_f/dist_scale),dist_scale)
        rover_dynamics.setup()

	#   Publications to ros topics
        traj_pub = rospy.Publisher('traj_to_low_RTD',TrajStamped,queue_size=100)

        #   Subscriptions to ros topics.
	#   Wait for enough info before proceeding to algorithm_main.

	vesc_sub = rospy.Subscriber('/vesc/odom',Odometry,callback_odom)
	rospy.wait_for_message('/vesc/odom',Odometry)

        grid_sub = rospy.Subscriber('map',OccupancyGrid,callback_oc)
	rospy.wait_for_message('map',OccupancyGrid)

	pose_raw = rospy.wait_for_message('cart_rover_global',RoverPoseGlobalStamped)
	pose_sub = rospy.Subscriber('cart_rover_global',RoverPoseGlobalStamped,callback_pose)
	rospy.wait_for_message('cart_rover_global',RoverPoseGlobalStamped)

        if planner:
            goal_sub = rospy.Subscriber('goal_to_mid_RTD',FloatStamped,callback_goal)
	    rospy.wait_for_message('goal_to_mid_RTD',FloatStamped)

	t_j = rospy.get_time()

	while not rospy.is_shutdown():
	    #   Loop
	    algorithm_main(cur_oc)	

	rospy.spin()

    except rospy.ROSInterruptException:
        pass


