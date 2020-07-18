#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import numpy as np
import matplotlib.pyplot as plt
if __name__ == '__main__':
    rospy.init_node('pose_listner')

    listener = tf.TransformListener()
    rate = rospy.Rate(1)
    cnt=0
    while not rospy.is_shutdown():
        try:
            (trans,quaternion) = listener.lookupTransform('world', 'wrist', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            continue
        #pose = np.array(pose)
        rot = tf.transformations.euler_from_quaternion(quaternion)
        #print("position {}".format(trans))
        #print("rotation {}".format(np.degrees(rot)))
        cnt+=1
        plt.scatter(cnt,np.degrees(rot[2]))
        plt.pause(0.05)
        
        rate.sleep()
