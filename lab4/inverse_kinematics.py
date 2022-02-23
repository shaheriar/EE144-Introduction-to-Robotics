import numpy as np
from math import pi, cos, sin, tan, atan, atan2, sqrt, acos

def inverse_kinematics(position):
    # input: the position of end effector [x, y, z]
    # output: joint angles [joint1, joint2, joint3]
    # add your code here to complete the computation

    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    x = position[0]
    y = position[1]
    z = position[2]

    #Finding theta 1
    theta1 = atan2(y,x)
    alpha = atan2(0.050,0.15)
    sqX = sqrt(x**2 + y**2)

    #Finding theta 2
    gamma = atan2(z-0.1039,sqX)
    hypo = sqrt(sqX**2 + (z-0.1039)**2)
    beta1 = acos(((0.1581**2) + hypo**2 - (0.15**2))/(2 * 0.1581 * hypo))
    theta2 = (pi/2) - alpha - beta1 - gamma

    #Finding theta 3
    beta2 = acos(((0.1581**2) + (0.15**2) - hypo**2)/(2 * 0.1581 * 0.15))   
    betaTemp = pi - beta2
    theta3 = pi - alpha - pi/2 - betaTemp

    joint1 = theta1
    joint2 = theta2
    joint3 = theta3

    return [joint1, joint2, joint3]