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
    goal_msg.header = Header(stamp=rospy.Time.now(),frame_id='base_imu_link')
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
	tol = 1
	goal = (0,0)

        goal_pub = rospy.Publisher('goal_to_mid_RTD',FloatStamped,queue_size=1)
        pose_sub = rospy.Subscriber('cart_rover_global',RoverPoseGlobalStamped,callback_pose)

        # Waypoint List as Array
        # 10_sep_19_hallway1.pbstream
	# A good guideline is to have the points spaced no more than 
	# the distance_scale in use as the solver can converge better
	# in the FRS space.
        way = np.array([[1.02,-11.97],
	    [1.26,-13.6],
	    [2.9,-13.5],
	    [4.34,-13.2],
            [6,-13],
	    [7.33,-12.8],
	    [9.02,-12.5],
	    [10.29,-12.6],
	    [12,-12.5],
	    [14.4,-12.4],
            [17.3,-11.88]])

	#hallways_obs_jan17.pbstream
        #way = np.array([[-2.03,-10.26],
	#    [0.878,-8.839],
	#    [0.44,-6.93],
	#    [0.46,-4.82],
        #    [0.775,-2.568],
	#    [0.0,0.0],
	#    [-1.27,-7.94]])

        while not rospy.is_shutdown():
            #   Loop
            calc_waypoint()
            rospy.Rate(loop_rate).sleep()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

