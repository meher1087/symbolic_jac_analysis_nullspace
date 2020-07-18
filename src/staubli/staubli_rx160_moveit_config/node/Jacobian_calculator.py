#!/usr/bin/env python

import sys
import rospy
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
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


# move robot to non singular position by adjusting joint vlaues in group
joint_goal = group.get_current_joint_values()
print(joint_goal)
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] =  -1
joint_goal[3] =  1

d2r = np.deg2rad
# q1 = d2r(np.linspace(-160, 160,10)) # desired range of motion for joint 1
# q2 = d2r(np.linspace(-137.5, 137.5,10)) # desired range of motion for joint 2
# q3 = d2r(np.linspace(-150,150,10)) # desired range of motion for joint 3
# q4 = d2r(np.linspace(-150,150,10)) # desired range of motion for joint 3

q1 = d2r(np.linspace(-100, 100,10)) # desired range of motion for joint 1
q2 = d2r(np.linspace(-50, 50,10)) # desired range of motion for joint 2
q3 = d2r(np.linspace(-50,50,10)) # desired range of motion for joint 3
#q4 = d2r(np.linspace(-50,50,10)) # desired range of motion for joint 3

g = np.meshgrid(q1,q2,q3)
grid = np.append(g[0].reshape(-1,1),g[1].reshape(-1,1),axis=1)
grid = np.append(grid,g[2].reshape(-1,1),axis=1)
#grid = np.append(grid,g[3].reshape(-1,1),axis=1)
# joint_goal=[-1.7, -0.9, -0.9, -0.9, 1, 0.9999606439766939]
# group.go(joint_goal,wait=True)
# group.stop()

eef_link = group.get_end_effector_link()

sample = open('jacob.txt', 'w')
fd =open('jacob.txt','w')
for i in grid:
    print(i)
    joint_goal = [i[0],i[1],i[2],0,0,0]
    print(joint_goal)
    group.go(joint_goal,wait=True)
    group.stop()

    # get pose
    pose = group.get_current_pose(eef_link).pose
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z

    jac = group.get_jacobian_matrix(joint_goal)
    jac_val = np.linalg.det(jac[:3,:3])
    val = [x,y,z,jac_val]
    fd.write(str(val))
    fd.write("\n")
fd.close()
