Status:
Can control robot and plot joint angles with python nodes in moveit_config package.

TODO:
1. Have to program Transformation matrix using symbolic math.
2. Get a path along which robot have singularities.
3. Need to include IK solution service in ROS
4. update the pose control node to use that service and apply pose.
5. get end effector pose and joint angle movements using tf ( modify plotting node)
