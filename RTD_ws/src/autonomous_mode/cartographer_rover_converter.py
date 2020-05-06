#! /usr/bin/env python

"""
    This code aims to decentralize a lot of the stuff the rover will inevitably have to deal with.

    More specifically, this script aims to read the mocap data and convert it to a format more suitable for the rover implementation.

    The desired format for the rover includes:
        -> x (m) belongs to (-inf, inf)
        -> y (m) belongs to (-inf, inf)
        -> psi (radians) belongs to (-inf, inf)
"""
import rospy
import message_filters
from autonomous_mode.msg import RoverPoseGlobalStamped
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from tf import transformations
import tf
from math import pi

rover_pose = rospy.Publisher('cart_rover_global', RoverPoseGlobalStamped, queue_size=1)
yaw_prev = 0
psi_global = 0

def tf_callback(translation, rotation):
    global yaw_prev     # Need to specify global variables used within each function.
    global psi_global
    
    x_pos = translation[0] #NEVER FORGET TO MAKE SURE UNITS ARE CORRECT!!1
    y_pos = -translation[1]

    quaternion_val = (
        rotation[0],
        rotation[1],
        rotation[2],
        rotation[3])
    euler_val = transformations.euler_from_quaternion(quaternion_val)
    yaw = -euler_val[2]
    
    yaw_diff = unwrap_angle_diff(yaw)

    # Update global heading angle.
    psi_global += yaw_diff
    
    # Update previous yaw value.
    yaw_prev = yaw

    # Make the message.
    rover_pose_msg = RoverPoseGlobalStamped()
    rover_pose_msg.header = Header(stamp=rospy.Time.now(), frame_id='base_link')
    rover_pose_msg.x = x_pos
    rover_pose_msg.y = y_pos
    rover_pose_msg.psi = psi_global

    rover_pose.publish(rover_pose_msg)

def unwrap_angle_diff(yaw_angle):
    """
        The logic is that if psi_global was near the cross-over point (-pi -> pi or pi -> -pi),
        the previous psi angle should be 
    """

    global yaw_prev     # Need to specify global variables used within each function.

    # Unwrap the angle based on previous value.
    if ((yaw_prev > 3) and (yaw_angle < 0)):
        yaw_angle += 2*pi
    elif ((yaw_prev < -3) and (yaw_angle > 0)):
        yaw_angle -= 2*pi

    # Find difference between current and previous angle.
    yaw_diff = yaw_angle - yaw_prev
    
    return yaw_diff


if __name__ == '__main__':
    try:
        rospy.init_node('rover_pose_global', anonymous=False)
        tf_listener = tf.TransformListener()
        
        # rospy.wait_for_service('spawn')
        # spaw

        rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            try:
                (trans, rot) = tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

                continue
            
            tf_callback(trans, rot)

            rate.sleep()

        # rospy.Subscriber('mocap', PoseStamped, mocap_callback)
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass    
