#! /usr/bin/env python

#-----------------------------------------------------------
#   The goal is the active waypoint. The waypoints are 
#   provided beforehand. The criteria to go to the next waypoint
#   is distance to current waypoint.
#-----------------------------------------------------------

import rospy
from std_msgs.msg import Header
from autonomous_mode.msg import RoverPoseGlobalStamped
from autonomous_mode.msg import FloatStamped
import numpy as np

#-----------------------------------------------------------
#   Calculate current waypoint
#-----------------------------------------------------------

def calc_waypoint():
    global curr

    dists = np.sqrt((goal[0]-way[:,0])**2+(goal[1]-way[:,1])**2)
    num_goals = len(way[:,0])

    for k in range(num_goals):
        #   New waypoint if close enough to current waypoint    
        if dists[k] < tol and curr == k and curr != num_goals-1:
            curr = k+1;

    goal_publisher()
    

#-----------------------------------------------------------
#   Publisher functions
#-----------------------------------------------------------

def goal_publisher():
    goal_msg = FloatStamped()
    goal_msg.header = Header(stamp=rospy.Time.now(),frame_id='base_link')
    goal_msg.data = [way[curr,0],way[curr,1]]
    goal_pub.publish(goal_msg)


#-----------------------------------------------------------
#   Callback functions
#-----------------------------------------------------------

def callback_pose(pose):
    global goal

    goal = (pose.x,-pose.y)

#-----------------------------------------------------------
#   main function
#-----------------------------------------------------------
if __name__ == '__main__':
    global way,curr,tol

    try:
        rospy.init_node('breadcrumbs',anonymous=False)

        loop_rate = 10
        curr = 0
	tol = 1.0
	goal = (0,0)

        goal_pub = rospy.Publisher('goal_to_mid_RTD',FloatStamped,queue_size=1)
        pose_sub = rospy.Subscriber('cart_rover_global',RoverPoseGlobalStamped,callback_pose)

        # Waypoint List as Array
        # 10_sep_19_hallway1.pbstream
	# A good guideline is to have the points spaced no more than 
	# the distance_scale in use as the solver can converge better
	# in the FRS space.
        #way = np.array([[1.02,-11.97],
	#    [1.26,-13.6],
	#    [2.9,-13.5],
	#    [4.34,-13.2],
        #    [6,-13],
	#    [7.33,-12.8],
	#    [9.02,-12.5],
	#    [10.29,-12.6],
	#    [12,-12.5],
	#    [14.4,-12.4],
        #    [17.3,-11.88]])

	#hallways_obs_jan17.pbstream
        #way = np.array([[-2.03,-10.26],
	#    [0.878,-8.839],
	#    [0.44,-6.93],
	#    [0.46,-4.82],
        #    [0.775,-2.568],
	#    [0.0,0.0],
	#    [-1.27,-7.94]])

        #jan26hallway.pbstream
        #way = np.array([[0.69,-10.49],
	#    [2.70,-7.73],
	#    [1.19,-5.37],
	#    [0.55,-2.46],
        #    [0.48,0.58],
	#    [-0.36,4.38],
	#    [-1.29,9.58]])

	#feb10_hallway.pbstream
        #way = np.array([[9.13,-7.67], #Landmark(0,17)
	#    [9.35,-4.07], #Landmark(0,12)
	#    [6.48,-1.97], #Landmark(0,40)
	#    [4.89,-2.63], #Landmark(0,44)
        #    [2.09,-0.35], #Landmark(0,39)
	#    [-0.02,0.68], #Landmakr(0,33)
	#    [-8.85,4.71]]) #Landmark(0,31)

	#washtenaw.pbstream (0.05 res)
	#way = np.array([[5.35,11.75], #Landmark(0,13)
	#    [6.39,13.13], #Landmark(0,14)
	#    [8.58,12.47], #Landmark(0,15)
	#    [8.09,14.9], #Landmark in between
        #    [9.98,17.10], #Landmark in between
	#    [6.95,19.27], #Landmark(0,18)
	#    [5.17,20.2], #Landmark(0,19)
	#    [3.97,21.95], #Landmark(0,20)
	#    [3.02,19.37], #Landmark(0,21)
	#    [3.99,18.45], #Landmark in between
        #    [3.36,17.43], #Landmark(0,22)
	#    [2.21,16.34], #Landmark(0,25)
	#    [0.73,14.06]]) #Landmark(0,26)

	#washtenaw_1.pbstream (0.1 res)
	#way = np.array([[3.93,11.87], #Landmark(0,15)
	#    [3.81,13.15], #in between
	#    [4.78,14.16], #Landmark(0,16)
	#    [5.42,15.25], #in between
	#    [6.00,15.90], #in between 
	#    [6.51,16.61], #Landmark(0.29)
	#    [7.45,18.47], #in between
        #    [6.38,19.53], #Landmark(0,30)
	#    [4.16,19.51], #in between
	#    [2.85,20.80], #Landmark(0,21)
	#    [0.56,20.61], #Landmark(0,23)
	#    [1.53,19.05], #in between
	#    [1.85,17.48], #in between
	#    [0.75,16.27], #Landmark(0,25)
	#    [-0.31,15.01], #Landmark(0,26)
	#    [0.06,13.33]]) #Landmark(0,27)

	#washtenaw_room.pbstream (0.1 res)
	way = np.array([[2.97,2.77], #Landmark(0,15)
	    [3.31,4.16], #in between
	    [3.12,5.05], #in between
	    [1.75,5.39], #Landmark(0,4)
	    [0.88,4.69], #in between 
	    [0.98,3.56], #in between
	    [0.42,2.17], #Landmark(0,9)

            [1.55,1.87]]) #in between

        while not rospy.is_shutdown():
            #   Loop
            calc_waypoint()
            rospy.Rate(loop_rate).sleep()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

