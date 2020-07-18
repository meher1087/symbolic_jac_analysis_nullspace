'''
Python code has T(0-4) matrix for positioning TX90 robot
It as sperical joint for end effector so position is independent
of last three joints (t4,t5,t6).  This code uses symbolic math on and findout
Jacobian of TX robot in symbols. Tried to create a grid owing limitations on joint angles
and apply on jacobian for unit q change and plot vectors across the grid to identify NULL SPACE.

But there is mistake in idea as Jacobian per unit change includes both negative and positive changes
in each dimension (dx could be +1 or =1) so its complex to plot all and observe NLL space thus 
code commented.

Instead Jacobian,and position vectors are writen to file call jacob.txt. 
Idea is to get joint angles from ROS and subs into these and validate the positions with graphics

Reference - Lu et al 2015 IK Analaysis paper

Objective is to move robot to position 
'''
import numpy as np
from sympy import *
from sympy.physics.mechanics import dynamicsymbols, msubs
import matplotlib.pyplot as plt

s = sin
c = cos
alpha,a,d,theta = symbols(['alpha','a','d','theta'])
q1,q2,q3,q4 = dynamicsymbols(['q1','q2','q3','q3'])

#q1,q2,q3,q4 = [0,pi/2,pi/2,pi]
#q1,q2,q3,q4 = [0,0,0,0]

#given DH parameter find where the origin is wrt end effector
# T = Matrix([ 
#     [cos(theta), sin(theta), 0, -a],
#     [-cos(alpha)*sin(theta), cos(alpha)*cos(theta), sin(alpha), -d*sin(alpha)],
#     [sin(alpha)*sin(theta), -sin(alpha)*cos(theta), cos(alpha),  -d*cos(alpha)],
#     [0,0,0,1] ])

# given DH parameter find where the end effector is wrt base
T = Matrix([ 
    [cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), -a*cos(theta)],
    [sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
    [0, sin(alpha), cos(alpha),  d],
    [0,0,0,1] ])


# D-H parameters
joint1 = { alpha:pi/2, a:150, d:550, theta:q1 }
joint2 = { alpha:0,    a:825, d:0,   theta:q2 }
joint3 = { alpha:pi/2, a:0,   d:0,   theta:q3 }
joint4 = { alpha:pi/2, a:0,   d:625, theta:q4 }

# joint5 = { alpha:pi/2, a:150, d:550, theta:q5 }
# joint6 = { alpha:pi/2, a:150, d:550, theta:q6 }

R1_2 = T.subs(joint1)  # rot and translate by join1 and link1
R2_3 = T.subs(joint2)  # rot and translate by join2 and link2
R3_4 = T.subs(joint3)  # rot and translate by join3 and link3
R4_5 = T.subs(joint4)  # rot and translate by join4 and link4

#R1_5 = R4_5*R3_4*R2_3*R1_2
R1_5 = R1_2*R2_3*R3_4*R4_5  # this transforms tool position to base coordinate frame
R = R1_5.evalf()
#R =  np.array(R.tolist()).astype(np.int32)

#position coordinates of end effector
px,py,pz = R[:3,-1] 

px = simplify(px)
py = simplify(py)
pz = simplify(pz)

# Jacobian
J11 = diff(px,q1)
J12 = diff(px,q2)
J13 = diff(px,q3)
#J14 = diff(px,q4)

J21 = diff(py,q1)
J22 = diff(py,q2)
J23 = diff(py,q3)
#J24 = diff(py,q4)

J31 = diff(pz,q1)
J32 = diff(pz,q2)
J33 = diff(pz,q3)
#J34 = diff(pz,q4)

J = Matrix([[J11,J12,J13],
            [J21,J22,J33], 
            [J31,J32,J33]
            ])

sym_j=simplify(J)

print(sym_j)
sample = open('jacob.txt', 'w') 
  
print(sym_j, file = sample)
print(px, file = sample)
print(py, file = sample)
print(pz, file = sample)

sample.close() 

# # simulate position
# fx = lambdify((q1,q2,q3), px, 'numpy')
# fy = lambdify((q1,q2,q3), py, 'numpy')
# fz = lambdify((q1,q2,q3), pz, 'numpy')

# fdx = lambdify((q1,q2,q3),J11+J12+J13,'numpy')
# fdy = lambdify((q1,q2,q3),J21+J22+J23,'numpy')
# fdz = lambdify((q1,q2,q3),J31+J32+J33,'numpy')

# import numpy as np

# d2r = np.deg2rad
# q1 = d2r(np.linspace(-160, 160,10)) # desired range of motion for joint 1
# q2 = d2r(np.linspace(-137.5, 137.5,10)) # desired range of motion for joint 2
# q3 = d2r(np.linspace(-150,150,10)) # desired range of motion for joint 2

# g = np.meshgrid(q1,q2,q3)
# grid = np.append(g[0].reshape(-1,1),g[1].reshape(-1,1),axis=1)
# grid = np.append(grid,g[2].reshape(-1,1),axis=1)

# xp = np.array(fx(q1,q2,q3))
# yp = np.array(fy(q1,q2,q3))
# zp = np.array(fz(q1,q2,q3))

# dx = np.array(fdx(grid[:,0],grid[:,1],grid[:,2]))
# dy = np.array(fdy(q1,q2,q3))
# dz = np.array(fdz(q1,q2,q3))

# # u = dx/np.sqrt(dx**2 + dy**2 + dz**2)
# # v = dy/np.sqrt(dx**2 + dy**2 + dz**2)
# # w = dz/np.sqrt(dx**2 + dy**2 + dz**2)

# # print(np.shape(dx))
# # print(np.shape(grid))
# # print(grid[:,0])

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# colors = np.where((dx<=0.0001)&(dx>=-0.0001),'#0,#1)
# pnt3d=ax.scatter(grid[:,0],grid[:,1],grid[:,2],c=colors)

# # cbar=plt.colorbar(pnt3d)
# # cbar.set_label("change in x")
# plt.show()


# # q = ax.quiver(x,y,z,u,v,w)  # plots arrows with x,y as point and u,v as directions
# # plt.show()

# #new = D*(np.transpose(position))
# #print(R)
# #print(px)

# # angles = {q1:0, q2:pi/2, q3:pi/2, q4:pi}
# # #angles = {q1:0, q2:0, q3:0, q4:0}

# # K = msubs(J,angles)

# # K = K.evalf()
# # K  = Matrix(K)
# # #E = K.eigenvects()
# # #E = J.eigenvals()
# # print(K)
# # N = K.nullspace()
# # print(N)

# # # D = det(J)
# # # D = simplify(D)
# # # N = solve(D,(q1,q2,q3))
# # print(E)