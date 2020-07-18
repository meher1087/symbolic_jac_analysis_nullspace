#!/usr/bin/env python

import sys
import rospy
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

#initialise moveit commander and rospy nodes
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

# instantiate Robotcommander object. This object is the outer inerface to the robot
robot = moveit_commander.RobotCommander()

# instantiate planningsceneinterfacae object. This object is an interface to the world surrounding the robot
scene=moveit_commander.PlanningSceneInterface()

#instantiate MoveGroupCommander for one of the planning groups of robot
group_name = "manipulator" #arm
group = moveit_commander.MoveGroupCommander(group_name)

#create a display trajectory publisher which is used later to publish trajectories for rvis to visulalize
display_trajectory_publisher=rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

# # We can get the name of the reference frame for this robot:
# planning_frame = group.get_planning_frame()
# print "============ Reference frame: %s" % planning_frame

# # We can also print the name of the end-effector link for this group:
# eef_link = group.get_end_effector_link()
# print "============ End effector: %s" % eef_link

# # We can get a list of all the groups in the robot:
# group_names = robot.get_group_names()
# print "============ Robot Groups:", robot.get_group_names()

# # Sometimes for debugging it is useful to print the entire state of the
# # robot:
# print "============ Printing robot state"
# print robot.get_current_state()
# print ""

# move robot to non singular position by adjusting joint vlaues in group
joint_goal = group.get_current_joint_values()
joint_goal[0] = -pi/2
joint_goal[1] = pi/4
joint_goal[2] = 0
joint_goal[3] = pi/4

#use go command to execute the joint angles
group.go(joint_goal,wait=True)

# call stop to avoid residual movements
group.stop()
