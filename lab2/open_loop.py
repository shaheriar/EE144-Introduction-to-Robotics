import rospy
from geometry_msgs.msg import Twist
from math import pi

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.run()


    def run(self):
        vel = Twist()
        vel.linear.x = 0.5
        vel.angular.z = 0

	rotate = Twist()
	rotate.linear.x = 0
	rotate.angular.z = 0.5 * pi
	#rotate.angular.z = 1.58

	count = 0

        #while not rospy.is_shutdown():  # uncomment to use while loop
	while (count < 4):  #to have the linear and angular velocity commands run 4 times
        	for i in range(80): #linear velocity for 80 ticks
            		self.vel_pub.publish(vel)
            		self.rate.sleep()
		rospy.loginfo('Linear movement complete')
		for i in range(10): #angular velocity to 10 ticks
			self.vel_pub.publish(rotate)
			self.rate.sleep()
		rospy.loginfo('Rotational movement complete\n')
		count += 1 #increment count
	rospy.loginfo('Visited all checkpoints. Closing.')

if __name__ == '__main__':
    try:
        whatever = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
