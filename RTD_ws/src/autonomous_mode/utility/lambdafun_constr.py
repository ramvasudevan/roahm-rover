import sys
import numpy as np
import sympy as syp
import dill
import scipy.io as sio


	
#------------------------------------------------------------------------------
# Various support functions for the constraint polynomial evaluations.
# Saves a custom power evaluation function as np.power is too slow
# in the IPOPT setting.
#
# Author of this script: Steven van Leeuwen (svanlee@umich.edu)
#------------------------------------------------------------------------------

def gen_pow_ind(p):
	leng = degree
	pow_ind = np.zeros([np.size(p,0),leng])
	for l in range(leng):
	    pow_ind[:,l] = np.where(p-l>0,1,0)
	return pow_ind


def setup_pow_eval(pow_ind):
	ret = syp.Matrix([1]*np.size(pow_ind,0))
	k = syp.symbols('k')
        for m in range(len(ret)):
	    for l in range(degree):
                if pow_ind[m,l] == 0:
                    continue
                else:
		    ret[m] = ret[m]*k*pow_ind[m,l]
	myfun = syp.lambdify(k,ret,'numpy')
	return myfun


# NOTE this script needs to be computed with a new FRS.
# NOTE make sure degree is correct.

global degree
degree = 10

frs = sio.loadmat(
            '/home/nvidia/RTD_ws/src/autonomous_mode/Mat_Files/FRS/rover_FRS_deg_10.mat')

p = frs['pows'].astype(float) 
w_k = np.squeeze(frs['k_cols'])
w_z = np.squeeze(frs['z_cols'])

print(p[:,w_k[0]])
k0_pow_ind = gen_pow_ind(p[:,w_k[0]])
k1_pow_ind = gen_pow_ind(p[:,w_k[1]])
k0_powm1_ind = gen_pow_ind(np.maximum(p[:,w_k[0]]-1,0))
k1_powm1_ind = gen_pow_ind(np.maximum(p[:,w_k[1]]-1,0))
z0_pow_ind = gen_pow_ind(p[:,w_z[0]])
z1_pow_ind = gen_pow_ind(p[:,w_z[1]])


fun1 = setup_pow_eval(k0_pow_ind)
fun2 = setup_pow_eval(k1_pow_ind)
fun3 = setup_pow_eval(k0_powm1_ind)
fun4 = setup_pow_eval(k1_powm1_ind)
fun5 = setup_pow_eval(z0_pow_ind)
fun6 = setup_pow_eval(z1_pow_ind)

print(fun1(2)) # Test raise 2 to the power ...

dill.settings['recurse'] = True
dill.dump(fun1,open('lambda_pow_k0','wb'))
dill.dump(fun2,open('lambda_pow_k1','wb'))
dill.dump(fun3,open('lambda_pow_m1_k0','wb'))
dill.dump(fun4,open('lambda_pow_m1_k1','wb'))
dill.dump(fun5,open('lambda_pow_z0','wb'))
dill.dump(fun6,open('lambda_pow_z1','wb'))



