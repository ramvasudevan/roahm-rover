#! /usr/bin/env python

#------------------------------------------------------------------------
# Author of this script: Steven van Leeuwen (svanlee@umich.edu)
#------------------------------------------------------------------------

import numpy as np


class solver_obj():
    #
    #   This class can be instantiated to create an object of the 
    #   appropriate form for the ipopt solver.
    #
 
    #--------------------------------------------------------------------
    #   Class constructor
    #--------------------------------------------------------------------

    def __init__(self,rover_dyn,oc_box_xy,w_p,w_c,w_k):
	#
	#   Information regarding constraints are initialized as zeros here.
	#
        self.rover_dyn = rover_dyn
	oc_pts = oc_box_xy[0].size*oc_box_xy[1].size
        self.p = np.zeros((oc_pts,np.size(w_p,0),np.size(w_p,1)))
        self.c = np.zeros((oc_pts,np.size(w_c)))
	self.c_tmp0 = np.zeros((oc_pts,np.size(w_c)))
	self.c_tmp1 = np.zeros((oc_pts,np.size(w_c)))
        self.k_cols = np.zeros(np.size(w_k),dtype=int)
        self.cnt = 0
	self.goal = [0,0]


    #---------------------------------------------------------------------
    #   Functions having to do with constraints
    #---------------------------------------------------------------------

    def add_constraint(self,p,c,k_cols):
	#
	#   Add an obstacle as a constraint. This equates to populating 
	#   along a dimension of p,c,k_cols.
	#
        self.p[self.cnt,:,:] = p
        self.c[self.cnt,:] = c
        self.k_cols[:] = k_cols
        self.cnt = self.cnt+1


    def finalize(self):
        #
        #   Trim extra zeros regarding constraints. Index the entries of 
        #   c where the power of k do not vanish when taking derivative.
        #
        self.p = self.p[range(self.cnt),:,:]
        self.c = self.c[range(self.cnt),:]
        self.c_tmp0 = self.c_tmp0[range(self.cnt),:]
        self.c_tmp1 = self.c_tmp1[range(self.cnt),:]
        jac_ind0 = np.where(self.p[:,:,self.k_cols[0]]-1 >= 0)
        jac_ind1 = np.where(self.p[:,:,self.k_cols[1]]-1 >= 0)
        self.c_tmp0 = 0*self.c_tmp0
	self.c_tmp1 = 0*self.c_tmp1	
        self.c_tmp0[jac_ind0] = self.c[jac_ind0]
        self.c_tmp1[jac_ind1] = self.c[jac_ind1]
	self.jac_return = np.zeros((self.cnt,2))


    def constraints(self,k):
        #
        #   Constraint function callable 
        #
        p = self.p
        c = self.c
        k_cols = self.k_cols

        w_eval = np.einsum('ij,ji->i',c,
            np.multiply(self.rover_dyn.lambda_pow_k0(k[0]),
	    self.rover_dyn.lambda_pow_k1(k[1])))
        return w_eval


    def jacobian(self,k):
        #
        #   Jacobian of constraint function callable
        #
        p = self.p
        c = self.c
        k_cols = self.k_cols
	c_tmp0 = self.c_tmp0
	c_tmp1 = self.c_tmp1	
	
        jac_eval0 = np.einsum('ij,ji->i',c_tmp0*p[:,:,k_cols[0]],
            np.multiply(self.rover_dyn.lambda_pow_m1_k0(k[0]),
	    self.rover_dyn.lambda_pow_k1(k[1])))
        jac_eval1 = np.einsum('ij,ji->i',c_tmp1*p[:,:,k_cols[1]],
            np.multiply(self.rover_dyn.lambda_pow_m1_k1(k[1]),
	    self.rover_dyn.lambda_pow_k0(k[0])))

	self.jac_return[:,0] = jac_eval0
	self.jac_return[:,1] = jac_eval1
	return self.jac_return


    #---------------------------------------------------------------------
    #   Functions having to do with cost function
    #---------------------------------------------------------------------

    def objective(self,k):
        #
        #   Callable cost function to minimize.
        #   This cost function is lowest when the ending point of the 
        #   realized trajectory is closest to waypoint specified.
        #   Compute the ending point using traj producing model
        #
        return self.rover_dyn.lambda_cost_l(k[0],k[1],self.goal[0],self.goal[1])


    def gradient(self,k):
        #
        #   Jacobian of cost function callable
        #
        return [self.rover_dyn.lambda_cost_jac0_l(k[0],k[1],self.goal[0],self.goal[1]),
            self.rover_dyn.lambda_cost_jac1_l(k[0],k[1],self.goal[0],self.goal[1])]


    def set_goal(self,goal):
	#
	#   Set the goal in the objective and gradient function
	#
	self.goal = goal
















