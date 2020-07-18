'''
Python code has T(0-4) matrix for positioning TX90 robot
It as sperical joint for end effector so position is independent
of last three joints (t4,t5,t6)

Reference - Lu et al 2015 IK Analaysis paper

Objective is to move robot to position 
'''
import numpy as np
from sympy import *

s = sin
c = cos
t1,t2,t3,t4 = symbols(['t1','t2','t3','t4'])
s1  = s(t1)
s2  = s(t2)
s3  = s(t3)
s4  = s(t4)

c1  = c(t1)
c2  = c(t2)
c3  = c(t3)
c4  = c(t4)

d1,d2,d3,d4,d6 = symbols(['d1','d2','d3','d4','d6'])

a1,a2,a3,a4 = symbols(['a1','a2','a3','a4'])

T = Matrix([ 
    [c1*c(t2+t3)*c4-s1*s4, -c1*s(t2+t3), -c1*c(t2+t3)*s4, d4*c1*s(t2+t3) - d3*s1 + a1*c1 + a2*c1*c2],
    [s1*c(t2+t3)*c4 +c1*s4, -s1*s(t2+t3), -s1*c(t2+t3)*s4 + c1*c4, d4*s1*s(t2+t3) + d3*c1 + a1*s1 + a2*c2*s1],
    [-s(t2+t3)*c4, -c(t2+t3), s(t2+t3)*s4,  d4*c(t2+t3) - a2*s2],
    [0,0,0,1] ])

q1,q2,q3,q4 = np.radians([56.31,-3.24,105.09,-1.08])

parameters = {
    t1:q1,
    t2:q2,
    t3:q3,
    t4:q4,

    a1:50,
    a2:650,
    
    d3:50,
    d4:650,
    d6:100}


D = T.subs(parameters)
D = D.evalf()
D =  np.array(D.tolist()).astype(np.int32)
position = D[:3,-1] 
#new = D*(np.transpose(position))
print(D)
print(position*0.1)


