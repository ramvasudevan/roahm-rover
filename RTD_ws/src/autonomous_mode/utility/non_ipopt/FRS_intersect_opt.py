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

import rospy
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from autonomous_mode.msg import TrajStamped
from autonomous_mode.msg import RoverPoseGlobalStamped
from rover_dynamics import rover_dynamics
import numpy as np
import scipy.optimize as sopt
import scipy.io as sio
import math

#------------------------------------------------------------------------------
#   These functions are support functions for the scipt, do not relate to 
#   the RTD pipeline
#------------------------------------------------------------------------------

def MSG_TIME(msg1=None,msg2=None,msg3=None):
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
    elif type(msg3) is np.ndarray:
        print '            Control commands:',msg3
        print '------'
        time_start = 0

#------------------------------------------------------------------------------
#   These functions have to do with processing the obstacle points.
#------------------------------------------------------------------------------

def gen_buf_discrete(oc_box_xy,oc_wh,oc_1,oc_prob,oc_d_expand,oc,xy0,res):
    #
    #   Generate occupancy points a certain spacing around each occupied point
    #   When the buffer generation is done, add the buffer points as constraints
    #   to the solver format.
    #
    obs = []

    #   Buffer each point registered with oc_discrete_expand.
    for x in oc_box_xy[0]:
        for y in oc_box_xy[1]:
            if oc_prob[x][y] >= oc_1:
                for k in range(-oc_d_expand,oc_d_expand+1):
                    if not (x-k < 0 or x+k >= oc_wh[0]): 
                        for l in range(-oc_d_expand,oc_d_expand+1):
                            if not (y-l < 0 or y+l >= oc_wh[1]):
                                oc[x+k][y+l] = 1

    #   Remove interior points and add to obstacle list.
    for x in oc_box_xy[0]:
        for y in oc_box_xy[1]:
            if oc[x][y] == 1:
                if (sum_surround_oc(x,y,oc,oc_wh) == 8):
                    continue
                else:
                    #   Put obstacles in FRS frame.
                    obs_FRS = obs_map_2_FRS(res*x+xy0[0],res*y+xy0[1],pose_future[0:3])

                    #   Append as a constraint if it is reachable in the FRS frame.
                    if obs_FRS[0] <= 1 and obs_FRS[1] <= 1:
                        obs.append(obs_FRS)
    return obs


def check_buf_discrete_ok(res,d,W):
    #
    #   It is possible the rover can pass far enough through the discretized buffer.
    #   Ensure r is smaller than rbar = footprint_W
    #   Ensure a is larger than bbar = footprint_W/2
    #
    r = res              
    a = r/np.sqrt(2)     
    if (r <= W):
        return 0
    buf = (d-1)*res+(res/2) 
    if (buf <= a):
        return 0
    return 1


def gen_oc_box_ind(ind,ind_expand,lim):
    #
    #   Make the list of indicies of the occupancy grid for 
    #   occupancy consideration.
    #
    oc_box = np.arange(max(ind-(ind_expand-1/2),0),
	min(ind+(ind_expand-1/2),lim-1))
    return oc_box


def sum_surround_oc(x,y,oc,oc_wh):
    #
    #   A utility function to gather whether a point is an interior point.
    #
    sum_ = 0
    if not (x < 1 or x >= oc_wh[0]-1):
        if not (y < 1 or y >= oc_wh[1]-1):
            sum_ = (oc[x-1][y-1]+oc[x-1][y]+oc[x-1][y]+oc[x][y-1]+
                oc[x][y+1]+oc[x+1][y-1]+oc[x+1][y]+oc[x+1][y+1])
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
    return (pose_frs[0],pose_frs[1])


def k_FRS_2_map(k):
    #
    #   Scale the k parameters to their control commands.
    #
    U = [0,0]
    U[0] = k[0]*k_scale[0]+0.75
    U[1] = k[1]*k_scale[1]
    return U


def cart_rover_global_2_map(pose):
    #
    #   Get pose into map frame. Additionally use current tracking
    #   trajectory to get velocity estimate.
    #
    try:
        t_ind = np.where(tpoints_traj_push > rospy.get_time()-t_traj_push) 
        return (pose.x,-pose.y,-pose.psi-(np.pi/2),Z_h_map[t_ind[0][0],4])
    except:
	return (pose.x,-pose.y,-pose.psi-(np.pi/2),Z_h_map[0,4])
	

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
    constr_in_k = []
    for k in range(n):
        z_0[k][:] = [obs[k][0],obs[k][1]]
        p_tmp = w_p.astype(float)
        c_tmp = w_c.astype(float)
        cnt = 0
        for l in w_z:

            #   Evaluate all powers of the current z variable in w
            p_tmp[:,l] = np.power(z_0[k][cnt],w_p[:,l])
            cnt = cnt+1

            #   Update coefficients of affected terms and collapse column
            #   of z variable just evaluated
            c_tmp = c_tmp*p_tmp[:,l]
            p_tmp[:,l] = np.zeros(np.size(p_tmp,0))

        #   Realize the obstacle point as a constraint and append to list.
        w_cur = constr_gen(p_tmp,c_tmp,w_k)
        constr_in_k.append(w_cur)
    return constr_in_k


def constr_gen(p,c,k_cols):
    #
    #   Construct the dictionary for the current obstacle point
    #   (format is specificed by solver)
    #   -For slsqp the constraint is in a dictinary format
    #   -The inequality constraint is a non-negative constraint
    #
    w_cur = {}
    w_cur['type'] = 'ineq'
    w_cur['args'] = (p,c,k_cols)
    w_cur['fun'] = constr_fun_w
    w_cur['jac'] = constr_jac_w
    return w_cur
        

def constr_fun_w(k,p,c,k_cols):
    #
    #   Constraint function callable 
    #
    w_eval = np.dot(c,np.power(k[0],p[:,k_cols[0]])*np.power(k[1],p[:,k_cols[1]]))
    return 1-w_eval


def constr_jac_w(k,p,c,k_cols):
    #
    #   Jacobian of constraint function callable
    #
    jac_eval = np.array([0,0])
    k0_ind = np.where(p[:,k_cols[0]]-1 >= 0)
    k1_ind = np.where(p[:,k_cols[1]]-1 >= 0)
    jac_eval[0] = np.dot(c[k0_ind]*p[k0_ind,k_cols[0]],
	np.squeeze(np.power(k[0],np.maximum(p[k0_ind,k_cols[0]]-1,0))
	    *np.power(k[1],p[k0_ind,k_cols[1]])))
    jac_eval[1] = np.dot(c[k1_ind]*p[k1_ind,k_cols[1]],
	np.squeeze(np.power(k[1],np.maximum(p[k1_ind,k_cols[1]]-1,0))
	    *np.power(k[0],p[k1_ind,k_cols[0]])))
    return -jac_eval


def constr_lb_k(k):
    #
    #   Constraint function callable
    #
    return 1-k


def constr_ub_k(k):
    #
    #   Constraint function callable
    #
    return 1+k


def constr_bound_k(solver_constr):
    #
    #   [-1,1] Bounds on k is also a constraint for the solver
    #
    lb_k = {}
    lb_k['type'] = 'ineq'
    lb_k['fun'] = constr_lb_k
    solver_constr.append(lb_k)
    ub_k = {}
    ub_k['type'] = 'ineq'
    ub_k['fun'] = constr_ub_k
    solver_constr.append(ub_k)
    return solver_constr
    

def J(k):
    #
    #   Callable cost function to minimize.
    #   This cost function is lowest when the ending point of the 
    #   realized trajectory is closest to waypoint specified.
    #   Compute the ending point using traj producing model
    #
    return rover_dynamics.lambda_cost_l(k[0],k[1],Z_FRS_goal[0],Z_FRS_goal[1])


def J_jac(k):
    #
    #   Jacobian of cost function callable
    #
    return [rover_dynamics.lambda_cost_jac0_l(k[0],k[1],Z_FRS_goal[0],Z_FRS_goal[1]),
        rover_dynamics.cost_jac1_l(k[0],k[1],Z_FRS_goal[0],Z_FRS_goal[1])]


def solver_opt_k_FRS(solver_constr):
    #
    #   Calls the solver to optimize over the cost function given constraints.
    #

    #   Solver options
    k_guess = [1,0]
    if use_jac == 'jac':
	jac_arg = J_jac
    else:
	jac_arg = None
    if print_:
        solver_option = {'ftol':1e-03,'maxiter':1e05,'disp':True}
    else:
        solver_option = {'ftol':1e-03,'maxiter':1e05,'disp':False}

    #   Based on solver timeout, decide if the rover should begin braking here.
    #   TODO

    #   Call solver
    result = sopt.minimize(J,k_guess,jac=jac_arg,method="slsqp",
	constraints=solver_constr,options=solver_option)
    return result


#------------------------------------------------------------------------------
#   This function is executed every time a new k is desired. All above 
#   functions are called through the callback_main.
#------------------------------------------------------------------------------

def callback_main(oc):
    #
    #   This function processes the occupancy grid from the environment
    #   and produces a trajectory to track.
    #
    global tpoints_traj_push,pose_future,ts_h,Z_FRS_goal,Z_h_map

    MSG_TIME(print_)

    #   Parse Occupancy Grid
    oc_wh = (oc.info.width,oc.info.height)
    oc_prob = np.reshape(oc.data,(oc_wh[0],oc_wh[1]))
    oc_res = oc.info.resolution
    oc_xy0 = (oc.info.origin.position.x,oc.info.origin.position.y)
    oc_ = np.zeros((oc_wh[0],oc_wh[1]),dtype=int)
    oc_rover_xy_ind = []
    oc_box_xy = []

    #   Below are ADJUSTABLE*********************************************
    oc_discrete_expand = 2  #   Distance in grid points for the buffer. 
                            #   For example = 1 registers the 8 surrounding 
                            #   grid points as occupied.
    t_delay = 0.25          #   Estimated computation delay (s) until rover actuated with new traj.
			    #   Must be multiple of ts_h.
    oc_box_size = 2.        #   Box area side length, (m), to register occupied grid points
    oc_1 = 50               #   Confidence level criteria of occupied grid point 0(none)-100(max)
    ts_h = 0.05             #   Time step for ref traj generation with the high fidelity dynamics
    #   End ADJUSTABLE***************************************************
    
    #--------------------------------------------------------------------
    #   Intersect obstacles and optimize over k
    #--------------------------------------------------------------------

    #   1. Find footprint of rover at later time, and form box to evaluate obstacles
    #   Estimate the future pose of rover, necessary because of computational delay

    #   Get pose in map frame, the same frame the occupancy grid is in
    pose_init = cart_rover_global_2_map(rospy.wait_for_message(
        'cart_rover_global',RoverPoseGlobalStamped))
    pose_future = tuple(rover_dynamics.flowf('high','no_record',pose_init,[0,t_delay],ts_h))

    #   Only looking at the occupancy grid in a conservative box area around the pose of rover
    #   will further reduce the computation burden
    oc_box_ind_expand = int(math.floor((oc_box_size/oc_res)))
    if oc_box_ind_expand%2 == 0:
        oc_box_ind_expand = oc_box_ind_expand+1

    for k in range(2):
        oc_rover_xy_ind.append(int(math.floor((pose_future[k]-oc_xy0[k])/oc_res)))
        oc_box_xy.append(gen_oc_box_ind(oc_rover_xy_ind[k],oc_box_ind_expand,oc_wh[k]))

    #   2. Buffer obstacles and obtain objects which to evaulate FRS against
    #   Rover is estimated to have a rectangular footprint, L is longitudinal, 
    #   W is lateral (m)
    if not check_buf_discrete_ok(oc_res,oc_discrete_expand,footprint_W):
	MSG_TIME(print_,'Discretization of buffer not ok')

    obs = gen_buf_discrete(oc_box_xy,oc_wh,oc_1,oc_prob,oc_discrete_expand,
	oc_,oc_xy0,oc_res)

    MSG_TIME(print_,'Generated buffer')

    #   3. Find the unsafe k parameters (constraint polynomials in k)
    solver_constr = constr_bound_k(proj_obs_to_k(obs))

    MSG_TIME(print_,'Generated constraints',len(solver_constr))

    #   4. Optimize 
    Z_FRS_goal = obs_map_2_FRS(Z_map_goal[0],Z_map_goal[1],pose_future)
    opt_result = solver_opt_k_FRS(solver_constr)

    MSG_TIME(print_,'Optimized')

    #   5. Realize the traj with chosen k
    opt_U = k_FRS_2_map(opt_result.x) 
    rover_dynamics.pass_U(opt_U)
    tpoints_traj_push = np.linspace(0,np.ceil(t_f),1+(np.ceil(t_f)/ts_h))
    X_h_map = rover_dynamics.flowf('high','record',pose_future,tpoints_traj_push,ts_h)

    #   Update Z_h_map and transform to roverinit frame 
    Z_h_map = np.concatenate((X_h_map,opt_U*np.ones((np.size(X_h_map,0),2))),axis=1)
    Z_h_roverinit = rover_dynamics.map_2_roverinit(Z_h_map) 

    #   Push traj_to_low_RTD
    traj_publisher(Z_h_roverinit) 

    MSG_TIME(print_,'Done',Z_h_roverinit[0,4:])


#------------------------------------------------------------------------------
#   This function publishes a new control command if one is obtained through
#   callback_main.
#------------------------------------------------------------------------------

def traj_publisher(Z):
    #
    #   Format the message that contains the trajectory to track, and publish. 
    #
    global t_traj_push

    traj_msg = TrajStamped()
    traj_msg_header = Header(stamp=rospy.Time.now(),frame_id='base_imu_link')
    traj_msg.ref_traj = Z.flatten()
    traj_msg.ref_size = np.size(Z[:,0],0) 
    traj_msg.Ts = ts_h;
    traj_pub.publish(traj_msg)
    t_traj_push = rospy.get_time()


#------------------------------------------------------------------------------
#   Update the current occupancy grid.
#------------------------------------------------------------------------------

def callback_oc(occupancy_grid):
    #
    #   This function gathers the occupancy grid from the environment
    #
    global cur_oc
   
    cur_oc = occupancy_grid


#------------------------------------------------------------------------------
#   Main
#------------------------------------------------------------------------------

if __name__ == '__main__':
    #
    #   Default main function. Will enter into callback_main.
    #
    global cur_oc,w_p,w_c,w_z,w_k,dist_scale,k_scale,use_jac,T,footprint_W,ts_l,\
        Z_h_map,Z_map_goal,time_start,t_traj_push,print_

    try:
        rospy.init_node('FRS_intersect_opt',anonymous=False)

	#   Initialize
	time_start = 0
        tpoints_traj_push = 0
        t_traj_push = np.inf
	Z_h_map = np.zeros((1,6))
	cur_oc = None

        #   Load FRS data structure
        #   -Assume not sparse
        #   -Assume w_z and w_k are each size 2, with the lower indexed column x,
        #   higher indexed column y for w_z
        frs = sio.loadmat(
            '/home/nvidia/RTD_ws/src/autonomous_mode/Mat_Files/FRS/rover_FRS_deg_6.mat')
        w_p = frs['pows'] 
        w_c = np.squeeze(frs['coef']) 
        w_z = np.squeeze(frs['z_cols'])
        w_k = np.squeeze(frs['k_cols'])
        dist_scale = np.squeeze(frs['dist_scale'])
	k_scale = np.squeeze(frs['k_scale'])
        t_f = np.squeeze(frs['t_f'])
        T = np.squeeze(frs['T'])
        footprint_W = np.squeeze(frs['footprint_W'])
        wb = np.squeeze(frs['wb'])
	coeffile = sio.loadmat(
            '/home/nvidia/RTD_ws/src/autonomous_mode/Mat_Files/newest_traj_lqr.mat')
	c = coeffile['coeffs']
	c = np.squeeze(c,0)

	#   Below are ADJUSTABLE**************************************************
	use_jac = None        #   Use jacobian associated with cost function.
                              #   Options are 'jac' or None.
	Z_map_goal = [-6.13,0.027]
                              #   Goal position for the rover. 
                              #   At where it would not move any further.
        print_ = True         #   Print elapsed time and diagnostic information to 
                              #   to standard out.
	entry_rate = 100      #   Rate between calls to callback_main in sec.
        #   End ADJUSTABLE********************************************************

	#   Create rover dynamics object
        rover_dynamics = rover_dynamics()
	rover_dynamics.pass_coeff(c,wb,(t_f/dist_scale))
        rover_dynamics.setup_pregen_cost_l()

	#   Publish to traj_to_low_RTD topic
        traj_pub = rospy.Publisher('traj_to_low_RTD',TrajStamped,queue_size=100)

        #   Subscribe to topic which publishes occupancy grid
        grid_sub = rospy.Subscriber('map',OccupancyGrid,callback_oc)

	cur_oc = rospy.wait_for_message('map',OccupancyGrid)

	while not rospy.is_shutdown():
	    #   Loop
	    callback_main(cur_oc)	
            rospy.Rate(entry_rate).sleep()

	rospy.spin()

    except rospy.ROSInterruptException:
        pass


