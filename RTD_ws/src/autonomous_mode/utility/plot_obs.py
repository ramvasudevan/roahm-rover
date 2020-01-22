#! /usr/bin/env python

#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
import sys
import numpy as np
import matplotlib.pyplot as plt
np.set_printoptions(threshold=sys.maxsize)

data = np.load('jan14.npy')
ind = np.where(data[:,0])
ind0 = ind[0][:-1]
indend = ind[0][-1]
plt.plot(data[ind0,0],data[ind0,1],'bo')
plt.plot(data[indend,0],data[indend,1],'ro')
plt.show()
plt.axis('equal')



    

