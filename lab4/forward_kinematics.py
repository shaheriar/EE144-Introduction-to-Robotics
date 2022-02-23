import numpy as np
from math import pi, cos, sin
import modern_robotics as mr

def forward_kinematics(joints):
    # input: joint angles [joint1, joint2, joint3]
    # output: the position of end effector [x, y, z]
    # add your code here to complete the computation

    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    joint1 = joints[0]
    joint2 = joints[1]
    joint3 = joints[2]

    M = [[1,0,0,link3x+link4x],
        [0,1,0,0],
        [0,0,1,link1z+link2z+link3z],
        [0,0,0,1]]

    w1 = np.array([0,0,1])
    w2 = np.array([0,1,0])
    w3 = np.array([0,-1,0])

    q1 = np.array([0,0,link1z])
    q2 = np.array([0,0,link1z+link2z])
    q3 = np.array([link3x,0,link3z+link1z+link2z])

    v1 = np.cross(-w1, q1)
    v2 = np.cross(-w2, q2)
    v3 = np.cross(-w3, q3)

    s1 = np.concatenate([w1, v1])
    s2 = np.concatenate([w2, v2])
    s3 = np.concatenate([w3, v3])
    
    s1 = mr.VecTose3(s1)
    s2 = mr.VecTose3(s2)
    s3 = mr.VecTose3(s3)

    e1 = mr.MatrixExp6(s1*joint1)
    e2 = mr.MatrixExp6(s2*joint2)
    e3 = mr.MatrixExp6(s3*joint3)

    T = np.matrix(e1)*np.matrix(e2)*np.matrix(e3)*np.matrix(M)

    x = T.item(0,3)
    y = T.item(1,3)
    z = T.item(2,3)

    return [x, y, z]
#Test case: joint1 = 0.52360, joint2 = -1.04720, joint3 = -0.52360
#print(forward_kinematics([0.52360,-1.04720,-0.52360]))
#print(forward_kinematics([pi/2,pi/2,0]))