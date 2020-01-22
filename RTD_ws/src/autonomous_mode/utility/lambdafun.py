#! /usr/bin/env python

#------------------------------------------------------------------------------
# Various support functions for the rover dynamics.
# Sets up the function to optimize over.
# Evaluated by calling a serialized function at runtime.
# 
# Author of this script: Steven van Leeuwen (svanlee@umich.edu)
#------------------------------------------------------------------------------

import numpy as np
import sympy as syp
import numpy as np
import dill

def obs_map_2_FRS(x,y,pose):
    #
    #   The FRS frame has the rover heading as zero heading, and in this 
    #   implementation at (0,0) position.
    #
    pose_xy = np.array([[x-pose[0]],[y-pose[1]]])
    R = np.array([[np.cos(pose[2]),np.sin(pose[2])],
        [-np.sin(pose[2]),np.cos(pose[2])]])
    pose_frs = (1/dist_scale)*np.matmul(R,pose_xy)
    return (pose_frs[1],pose_frs[0])

class lambdafun:
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


    def setup_pregen_flowf_l(self,tpoints,h,k_scl,jac):
        #
        #   Use the symbolic toolbox to generate expression for final endpoint
        #   of a trajectory, and then convert the symbolic expression to 
        #   sympy lambda function.
        #
        k_0,k_1,Zg_0,Zg_1 = syp.symbols('k_0,k_1,Zg_0,Zg_1')
        X_0 = 0
        X_1 = 0
	wb = self.wb
        dyn_scl = self.scale*h
	dist_scl = self.dist_scale
        n = int(1/h)
        for k in range(n):
	    w = dist_scl*(k_scl[0]*k_0+1)*(k_scl[1]*k_1)/wb
            dX_0 = (k_scl[0]*k_0+1)-w*X_1
            dX_1 = w*X_0
            X_0 = X_0+dyn_scl*dX_0
	    X_1 = X_1+dyn_scl*dX_1
        expr = (X_0-Zg_0)**2+(X_1-Zg_1)**2
        self.pregen_flowf_l = syp.lambdify([k_0,k_1,Zg_0,Zg_1],expr,'numpy')
	if jac == 'jac':
	    expr_jac0 = syp.diff(expr,k_0)
	    expr_jac1 = syp.diff(expr,k_1)
            self.pregen_flowf_jac0_l = syp.lambdify([k_0,k_1,Zg_0,Zg_1],expr_jac0,'numpy')
	    self.pregen_flowf_jac1_l = syp.lambdify([k_0,k_1,Zg_0,Zg_1],expr_jac1,'numpy')
    

myfun = lambdafun()
c = [1.6615e-05,-1.9555e-07,3.619e-06,4.3820e-07,-0.0811,-1.4736,0.1257,0.0765,-0.0140]
wb = 0.32
t_f = 1.17
dist_scale = 2.89

myfun.pass_coeff(c,wb,(t_f/dist_scale),dist_scale)

ts_l = 0.1
T = 1
k_scale = [1,0.34]

myfun.setup_pregen_flowf_l([0,T],ts_l,k_scale,'jac')

dill.settings['recurse'] = True
dill.dump(myfun.pregen_flowf_l,open('lambda_cost_l','wb'))
dill.dump(myfun.pregen_flowf_jac0_l,open('lambda_cost_jac0_l','wb'))
dill.dump(myfun.pregen_flowf_jac1_l,open('lambda_cost_jac1_l','wb'))




    

