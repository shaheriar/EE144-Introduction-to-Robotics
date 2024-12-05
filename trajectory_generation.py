
#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from motion_planning import get_path_from_A_star

class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        # calculate P_term and D_term
        error = self.set_point - current_value
        P_term = self.Kp*error
        D_term = self.Kd*(error - self.previous_error)   #idk
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0
    
    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()
        
        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        #-----------------------------------------------
        self.previous_waypoint = [0,0]  #x, y
        self.previous_velocity = [0,0]  #vx, vy
        self.vel_ref = 0.3
        self.vel = Twist()
        #-----------------------------------------------

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')


    def run(self):
        start = [0,2]
        goal = [9,2]
        obstacles = [[2,1],[2,2],[5,2],[5,3]]
        waypoints = get_path_from_A_star(start, goal, obstacles)
        for i in range(len(waypoints)-1):
            self.move_to_point(waypoints[i], waypoints[i+1])


    def move_to_point(self, current_waypoint, next_waypoint):
        # generate polynomial trajectory and move to current_waypoint
        # determine boundary conditions for the position    -   boundary for x and y
        px_start = self.previous_waypoint[0]
        px_end = current_waypoint[0]

        py_start = self.previous_waypoint[1]
        py_end = current_waypoint[1]

        # determine boundary conditions for the velocity    -   velocity for vx and vy
        vx_start = self.previous_velocity[0]
        vy_start = self.previous_velocity[1]

        # dx = x1 - x0
        # decompose the velocity
        # vx = v ref * cos(angle)
        # vy = v ref * sin(angle)
        dx = next_waypoint[0] - current_waypoint[0]
        dy = next_waypoint[1] - current_waypoint[1]
        angle = atan2(dx,dy)
        
        vx_end = self.vel_ref*cos(angle)
        vy_end = self.vel_ref*sin(angle)

        T = 2

        # Tx = dx/(vx_end-vx_start)
        # Ty = dy/(vy_end-vy_start)
        # T = int(2*sqrt(dx**2 + dy**2)/0.3)

        ax = self.polynomial_time_scaling_3rd_order(px_start, vx_start, px_end, vx_end, T) #test diff T's
        ay = self.polynomial_time_scaling_3rd_order(py_start, vy_start, py_end, vy_end, T) #test diff T's

        # x(t) = a0 + (a1 * t) + (a2 * t**2) + (a3 * t**3)

        # v(t)= a1 + (2 * a2 * t) + (3 * a3 * t**2)
        
        ctrl = Controller()
        ctrl.setPD(5,0)

        for i in range(0,9*T):
            t = i*0.1
            vx_end = np.dot([3*(t**2), 2*t, 1, 0],ax)
            vy_end = np.dot([3*(t**2), 2*t, 1, 0],ay)
            # self.vel.linear.x = ax[2] + (2 * ax[1] * t) + (3 * ax[0] * t**2)
            # self.vel.linear.y = ay[2] + (2 * ay[1] * t) + (3 * ay[0] * t**2)
            #compute theta
            theta = atan2(vy_end,vx_end)
            dt = theta - self.pose.theta

            ctrl.setPoint(theta)
            if (dt > pi):
                self.vel.angular.z = ctrl.Kp*(dt-(2*pi))
            elif(dt < -pi):
                self.vel.angular.z = ctrl.Kp*(dt+(2*pi))
            else:
                self.vel.angular.z = ctrl.Kp*dt

            self.vel.linear.x = sqrt(vx_end**2 + vy_end**2)
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
            #compute deviation bw theta and self.pose2D.theta   -   D_theta
            #update velocity
            # vel = Kp * delta theta (use high values of Kp)
            #use PID controller and use Vx_end and Vy_end to compute deviation and publish angular z
            #publish linear.x (magnitude of v) and angular.z (Kp * D_theta)
        self.previous_waypoint = current_waypoint
        self.previous_velocity = [vx_end,vy_end]
        pass


    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        # input: p,v: position and velocity of start/end point
        #        T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial
        x = np.array([p_start,p_end,v_start,v_end])
        M = np.array([[0,0,0,1],[T**3, T**2, T, 1],[0,0,1,0],[3*(T**2),2*T,1,0]])
        return np.dot(np.linalg.inv(M),x)

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
    whatever = Turtlebot()
