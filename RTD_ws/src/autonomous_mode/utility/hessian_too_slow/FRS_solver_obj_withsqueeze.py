#! /usr/bin/env python

#------------------------------------------------------------------------
# Author of this script: Steven van Leeuwen (svanlee@umich.edu)
#------------------------------------------------------------------------

import numpy as np
import scipy


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
        self.c_tmp00 = self.c_tmp0[range(self.cnt),:]
        self.c_tmp01 = self.c_tmp1[range(self.cnt),:]
        self.c_tmp11 = self.c_tmp1[range(self.cnt),:]
        m1_ind0 = np.where(self.p[:,:,self.k_cols[0]]-1 >= 0)
        m1_ind1 = np.where(self.p[:,:,self.k_cols[1]]-1 >= 0)
        m2_ind0 = np.where(self.p[:,:,self.k_cols[0]]-2 >= 0)
        m2_ind1 = np.where(self.p[:,:,self.k_cols[1]]-2 >= 0)
        self.c_tmp0 = 0*self.c_tmp0
	self.c_tmp1 = 0*self.c_tmp1	
        self.c_tmp00 = 0*self.c_tmp00
        self.c_tmp01 = 0*self.c_tmp01
	self.c_tmp11 = 0*self.c_tmp11	
        self.c_tmp0[m1_ind0] = self.c[m1_ind0]
        self.c_tmp1[m1_ind1] = self.c[m1_ind1]
        self.c_tmp00[m2_ind0] = self.c[m2_ind0]
        self.c_tmp01[m1_ind0 and m1_ind1] = self.c[m1_ind0 and m1_ind1] 
        self.c_tmp11[m2_ind1] = self.c[m2_ind1]
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


    def hessianstructure(self):
	global hs
	hs = scipy.sparse.coo_matrix(np.tril(np.ones((2,2))))
        return (hs.col,hs.row)

    def hessian(self,k,lagrange,obj_factor):
        #
        #   Hessian of the Lagrangian callable, size (2x2), returned as (4x1)
        #
        p = self.p
        k_cols = self.k_cols
	c_tmp00 = self.c_tmp00
 	c_tmp01 = self.c_tmp01
	c_tmp11 = self.c_tmp11	
        
        #   Objective term
        H = np.squeeze(obj_factor*\
            (np.array([[self.rover_dyn.lambda_cost_hess00_l(\
            k[0],k[1],self.goal[0],self.goal[1]),\
            self.rover_dyn.lambda_cost_hess01_l(\
            k[0],k[1],self.goal[0],self.goal[1])],\
            [self.rover_dyn.lambda_cost_hess10_l(\
            k[0],k[1],self.goal[0],self.goal[1]),\
            self.rover_dyn.lambda_cost_hess11_l(\
            k[0],k[1],self.goal[0],self.goal[1])]])))

	print(H.shape)
        #   Constraints terms 
        symm_term = np.dot(lagrange,np.einsum('ij,ji->i',c_tmp01*p[:,:,k_cols[0]]*
                p[:,:,k_cols[1]],np.multiply(self.rover_dyn.lambda_pow_m1_k0(k[0]),
                self.rover_dyn.lambda_pow_m1_k1(k[1]))))

        H += np.squeeze(np.array([[\
            np.dot(lagrange,np.einsum('ij,ji->i',c_tmp00*p[:,:,k_cols[0]]*
                (p[:,:,k_cols[0]]-1),np.multiply(self.rover_dyn.lambda_pow_m2_k0(k[0]),
                self.rover_dyn.lambda_pow_k1(k[1])))),symm_term],[symm_term,\
            np.dot(lagrange,np.einsum('ij,ji->i',c_tmp11*p[:,:,k_cols[1]]*
                (p[:,:,k_cols[1]]-1),np.multiply(self.rover_dyn.lambda_pow_m2_k1(k[1]),
                self.rover_dyn.lambda_pow_k0(k[0]))))]]))

        return H[hs.row,hs.col]



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
        #   Jacobian of cost function callable size (2x1)
        #
        return [self.rover_dyn.lambda_cost_jac0_l(k[0],k[1],self.goal[0],self.goal[1]),
            self.rover_dyn.lambda_cost_jac1_l(k[0],k[1],self.goal[0],self.goal[1])]


    def set_goal(self,goal):
	#
	#   Set the goal in the objective and gradient function
	#
	self.goal = goal
















