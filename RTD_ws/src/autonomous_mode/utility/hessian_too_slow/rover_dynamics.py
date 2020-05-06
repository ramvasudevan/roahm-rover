#! /usr/bin/env python

#------------------------------------------------------------------------------
# Various support functions for the rover dynamics.
#
# Author of this script: Steven van Leeuwen (svanlee@umich.edu)
#------------------------------------------------------------------------------

import numpy as np
import sympy as syp
import dill

class rover_dynamics:
    #
    #   Class that computes future poses for the rover.
    #

    #--------------------------------------------------------------------------
    #   Class variables
    #--------------------------------------------------------------------------
    c = 0
    wb = 0
    U = np.zeros(2)


    #--------------------------------------------------------------------------
    #   These functions update class variables
    #--------------------------------------------------------------------------

    def pass_coeff(self,c_in,wb_in,scale_to_1_in,dist_scale_in):
        #
        #   Pass in coefficients for high level model, and wheel base of rover.
        #
        self.c = c_in
        self.wb = float(wb_in)
        self.scale = float(scale_to_1_in)
	self.dist_scale = float(dist_scale_in)


    def pass_U(self,U_in):
        #
        #   Pass in current control input.
        #   U[0] is longitudinal velocity command, U[1] is steering angle command.
        #
        self.U = U_in


    #   -------------------------------------------------------------------------
    #   These functions have to do with dynamics, jacobians, hessians
    #   -------------------------------------------------------------------------

    def flowf(self,fun,rec,X0,tpoints,h):
        #
        #   Explicit forward Euler integration. Allows for recording at time points.
        #
        X_record = np.zeros((len(tpoints),len(X0)))
        X = np.asarray(X0)

        n = int(tpoints[-1]*(1/h))+1
        for k in range(n):
	    if rec == 'record':
                if (k*h == tpoints[k]):
                    X_record[k,:] = X

            if fun == 'low': 
                X = X+h*self.dx_l(X)
            elif fun == 'high':
                X = X+h*self.dx_h(X)

        if rec == 'record':
	    return X_record
	return X
 

    def setup(self):
        #
        #   Use the symbolic toolbox to generate expression for final endpoint
        #   of a trajectroy, and then convert the symbolic expression to non-symbolic
        #   for computation without for loops.
        #
        #   The following code is an example of what is also generated offline
	#
        #   See lambda* scripts in the utility folder.

        self.lambda_cost_l = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_cost_l','rb'))
	self.lambda_cost_jac0_l = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_cost_jac0_l','rb'))
	self.lambda_cost_jac1_l = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_cost_jac1_l','rb'))
	self.lambda_cost_hess00_l = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_cost_hess00_l','rb'))
	self.lambda_cost_hess01_l = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_cost_hess01_l','rb'))
	self.lambda_cost_hess10_l = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_cost_hess10_l','rb'))
	self.lambda_cost_hess11_l = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_cost_hess11_l','rb'))
        self.lambda_pow_k0 = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_pow_k0','rb'))
	self.lambda_pow_m1_k0 = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_pow_m1_k0','rb'))
	self.lambda_pow_m2_k0 = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_pow_m2_k0','rb'))
	self.lambda_pow_k1 = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_pow_k1','rb'))
        self.lambda_pow_m1_k1 = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_pow_m1_k1','rb'))
        self.lambda_pow_m2_k1 = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_pow_m2_k1','rb'))
	self.lambda_pow_z0 = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_pow_z0','rb'))
        self.lambda_pow_z1 = dill.load(open(
	    '/home/nvidia/RTD_ws/src/autonomous_mode/utility/lambda_pow_z1','rb'))
   
    

    def dx_l(self,X):
        #
        #   Evaluate derivative of state at passed in X and current U
        #
        dX = np.zeros(2)

        w = U[0]*U[1]/self.wb
        dX[0] = U[0]-(w*X[1])
        dX[1] = w*X[0]
        return dX


    def dx_h(self,X):
        #
        #   Evaluate derivative of state at passed in X and current U
        #
	c = self.c
	U = self.U
        dX = np.zeros(4)

        dX[2] = (np.tan(c[0]*U[1]+c[1])*X[3])/(c[2]+c[3]*X[3]**2)
        v_y = dX[2]*(c[7]+c[8]*X[3]**2) 
        dX[0] = X[3]*np.cos(X[2])-v_y*np.sin(X[2])
        dX[1] = X[3]*np.sin(X[2])+v_y*np.cos(X[2])
        dX[3] = c[4]+c[5]*(X[3]-U[0])+c[6]*(X[3]-U[0])**2  
        return dX


    def jacobian_eval_h(self,Z,ind):
        #
        #   Here Z is of format [X U]', and is the entire ref trajectory.
        #
	c = self.c
        A = np.zeros((4,4))
        B = np.zeros((4,2))

        A[0,2,ind] = (-Z[ind,3]*np.sin(Z[ind,2])-(np.tan(c[0]*Z[ind,5]+c[1])*Z[ind,3]/
            (c[2]+c[3]*Z[ind,3]**2))*(c[7]+c[8]*Z[ind,3]**2)*np.cos(Z[ind,2]))
        A[0,3,ind] = (np.cos(Z[ind,2])-np.tan(c[0]*Z[ind,5]+c[1])*(c[7]*np.sin(Z[ind,2]))*
            ((c[2]-c[3]*Z[ind,3]**2)/((c[2]+c[3]*Z[ind,3]**2)**2))
	    -np.tan(c[0]*Z[ind,5]+c[1])*(c[8]*np.cos(Z[ind,2]))*((3*c[2]*Z[ind,3]**2+c[3]*
	    Z[ind,3]**4)/((c[2]+c[3]*Z[ind,3]**2)**2)))
        A[1,2,ind] = (Z[ind,3]*np.cos(Z[ind,2])-(np.tan(c[0]*Z[ind,5]+c[1])*Z[ind,3]/
            (c[2]+c[3]*Z[ind,3]**2))*(c[7]+c[8]*Z[ind,3]**2)*np.sin(Z[ind,2]))
        A[1,3,ind] = (np.sin(Z[ind,2])+np.tan(c[0]*Z[ind,5]+c[1])*(c[7]*np.cos(Z[ind,2]))*
            ((c[2]-c[3]*Z[ind,3]**2)/((c[2]+c[3]*Z[ind,3]**2)**2))+np.tan(c[0]*Z[ind,5]+c[1])*
            (c[8]*np.cos(Z[ind,2]))*((3*c[2]*Z[ind,3]**2+c[3]*Z[ind,3]**4)/
	    ((c[2]+c[3]*Z[ind,3]**2)**2)))
        A[2,3,ind] = (np.tan(c[0]*Z[ind,5]+c[1])*(c[2]-c[3]*(Z[ind,3]**2))/
	    ((c[2]+c[3]*(Z[ind,3]**2))**2))
        A[3,3,ind] = c[5]+2*c[6]*(Z[ind,3]-Z[ind,4])
        B[0,1,ind] = ((-c[0]*Z[ind,3]*(1/(np.cos(c[0]*Z[ind,5]+c[1])**2))/
            (c[2]+c[3]*(Z[ind,3]**2)))*(c[7]+c[8]*Z[ind,3]**2)*np.sin(Z[ind,2]))
        B[1,1,ind] = ((c[0]*Z[ind,3]*(1/(np.cos(c[0]*Z[ind,5]+c[1])**2))/
            (c[2]+c[3]*(Z[ind,3]**2)))*(c[7]+c[8]*Z[ind,3]**2)*np.cos(Z[ind,2]))
        B[2,1,ind] = c[0]*Z[ind,3]*(1/(np.cos(c[0]*Z[ind,5]+c[1])**2))/(c[2]+c[3]*(Z[ind,3]**2))
        B[3,0,ind] = -c[5]-2*c[6]*(Z[ind,3]-Z[ind,4])
        return (A,B)


    #----------------------------------------------------------------------------
    #   These functions have to do with transformations
    #----------------------------------------------------------------------------

    def roverinit_2_map(self,X,psi):
	#
        #   Convert a pose from the roverinit frame, given the map psi value, 
	#   to the map frame.
        #
        H = np.array([[np.cos(psi),np.sin(psi)],[-np.sin(psi),np.cos(psi)]])
	xy_map = np.matmul(H,np.array([X[0],X[1]]))
	X_map = np.array([xy_map[0],xy_map[1],psi-X[2],X[3]])
	return X_map





    

