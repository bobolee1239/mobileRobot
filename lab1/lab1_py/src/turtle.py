#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
PI   = math.pi

class turlteBot:
    
    def __init__(self):
        rospy.init_node('robot_cleaner', anonymous=True)
        self._velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self._vel_msg = Twist()
        self._rate = rospy.Rate(10)

    def move(self, speed, distance, isForward):
        print("Let's move your robot")
    
        #Checking if the movement is forward or backwards
        if(isForward):
            self._vel_msg.linear.x = abs(speed)
        else:
            self._vel_msg.linear.x = -abs(speed)
        #Since we are moving just in x-axis
        self._vel_msg.linear.y  = 0
        self._vel_msg.linear.z  = 0
        self._vel_msg.angular.x = 0
        self._vel_msg.angular.y = 0
        self._vel_msg.angular.z = 0

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance and not rospy.is_shutdown()):
            #Publish the velocity
            self._velocity_publisher.publish(self._vel_msg)
            #Takes actual time to velocity calculus
            t1 = rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance = speed*(t1-t0)

        #Force the robot to stop
        self._vel_msg.linear.x = 0
        self._velocity_publisher.publish(self._vel_msg)

    def rotate(self, speed, angle, clockwise):
        print("Let's rotate your robot")

        #Converting from angles to radians
        angular_speed = speed*2*PI/360
        relative_angle = angle*2*PI/360

        #We wont use linear components
        self._vel_msg.linear.x  = 0
        self._vel_msg.linear.y  = 0
        self._vel_msg.linear.z  = 0
        self._vel_msg.angular.x = 0
        self._vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise:
            self._vel_msg.angular.z = -abs(angular_speed)
        else:
            self._vel_msg.angular.z = abs(angular_speed)

        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle and not rospy.is_shutdown()):
            self._velocity_publisher.publish(self._vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        #Forcing our robot to stop
        self._vel_msg.angular.z = 0
        self._velocity_publisher.publish(self._vel_msg)

    def sleep(self):
        self._rate.sleep()

if __name__ == '__main__':
    # Starts a new node
    turtleBoss = turlteBot()

    #Testing our function
    count = 1
    while not rospy.is_shutdown():
        if(count % 3): turtleBoss.move(1, 1, True)
	else: turtleBoss.rotate(30, 90, False)
        turtleBoss.sleep()
	count += 1
