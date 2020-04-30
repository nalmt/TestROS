#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt
PI = 3.1415926535897

class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def move2goal(self):
        goal_pose = Pose()
        goal_pose.x = input("Set your x goal:")
        goal_pose.y = input("Set your y goal:")
        distance_tolerance = input("Set your tolerance:")
        vel_msg = Twist()


        while sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) >= distance_tolerance:

            #Porportional Controller
            #linear velocity in the x-axis:
            vel_msg.linear.x = 1.5 * sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 4 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)

        #rospy.spin()

    def move(self, speed = 2, distance = 1, isForward = 1):
        # Receiveing the user's input
        vel_msg = Twist()

        # Checking if the movement is forward or backwards
        if (isForward):
            vel_msg.linear.x = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)
        # Since we are moving just in x-axis
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        #while not rospy.is_shutdown():
        # Setting the current time for distance calculus
        t0 = float(rospy.Time.now().to_sec())
        current_distance = 0

        # Loop to move the turtle in an specified distance
        while (current_distance < distance):
            # Publish the velocity
            self.velocity_publisher.publish(vel_msg)
            # Takes actual time to velocity calculus
            t1 = float(rospy.Time.now().to_sec())
            # Calculates distancePoseStamped
            current_distance = speed * (t1 - t0)
        # After the loop, stops the robot
        vel_msg.linear.x = 0
        # Force the robot to stop
        self.velocity_publisher.publish(vel_msg)

        print("X: % 2.1f, Y: % 2.1f" % (self.pose.x, self.pose.y))

        if self.pose.x > 0.1 and self.pose.x < 10.9 and self.pose.y > 0.1 and self. pose.y < 10.9:
            return 0
        else:
            return 1

    def rotate(self, speed = 70, angle = 70, clockwise = 0):
        # Receiveing the user's input
        vel_msg = Twist()

        # Converting from angles to radians
        angular_speed = speed * 2 * PI / 360
        relative_angle = angle * 2 * PI / 360

        # We wont use linear components
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while (current_angle < relative_angle):
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)

        # Forcing our robot to stop
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        # rospy.spin()

        return 0

    def feedback(self, action):
        if action == 0:
            return self.move()
        else:
            return self.rotate()


if __name__ == '__main__':
    try:
        #Testing our function
        choice = input("0: move to goal, 1: move forward, 2: rotate, 3: interaction: ")
        x = turtlebot()
        if choice == 0:
            x.move2goal()
        elif choice == 1:
            print("Let's move your robot")
            speed = input("Input your speed:")
            distance = input("Type your distance:")
            isForward = input("Foward?: ")
            x.move(speed, distance, isForward)
        elif choice == 2:
            print("Let's rotate your robot")
            speed = input("Input your speed (degrees/sec):")
            angle = input("Type your distance (degrees):")
            clockwise = input("Clowkise?: ")  # True or false
            x.rotate(speed, angle, clockwise)
        else:
            print("Let's make the robot interact")
            interaction = 1
            while interaction > 0:
                interaction = input("0: move, 1:rotate, 2: stop: ")
                if interaction == 0:
                    feedback = x.move()
                    print("Feedback: % 1d" % feedback)
                elif interaction == 1:
                    x.rotate()
                    print("Feedback: 0")
                else:
                    print("Stop")


    except rospy.ROSInterruptException: pass
