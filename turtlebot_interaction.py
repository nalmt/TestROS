#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Turtlebot:

    def __init__(self):
        # Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    # Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def move(self, linear_speed=0.0, angular_speed=0.0, duration=1.0) -> int:
        vel_msg = Twist()

        vel_msg.linear.x = linear_speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular_speed

        # Setting the current time for duration calculus
        t0 = float(rospy.Time.now().to_sec())

        # Loop to move the turtle during a specific duration
        while float(rospy.Time.now().to_sec()) - t0 < duration:
            # Publish the velocity
            self.velocity_publisher.publish(vel_msg)

        # After the loop, stops the robot
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        # Publish the null velocity to force the robot to stop
        self.velocity_publisher.publish(vel_msg)

        print("X: % 2.1f, Y: % 2.1f" % (self.pose.x, self.pose.y))

        # return feedback 1 if position is against the wall
        if 0.1 < self.pose.x < 10.9 and 0.1 < self.pose.y < 10.9:
            return 0
        else:
            return 1

    def feedback(self, action: int) -> int:
        if action == 0:
            # move forward
            return self.move(linear_speed=1)
        elif action == 1:
            # rotate left
            return self.move(linear_speed=0.1, angular_speed=1)
        elif action == 2:
            # rotate right
            return self.move(linear_speed=0.1, angular_speed=-1)
        else:
            return 0


if __name__ == '__main__':
    try:
        x = Turtlebot()
        print("Let's move the robot ")
        choice = input("Type 0 to enter values or 1 to enter interactions: ")
        if choice == 0:
            lx_speed = float(input("Input the linear speed (cell /sec): "))
            az_speed = float(input("Input the angular speed (rad /sec): "))
            d = float(input("Input the duration (sec): "))
            feedback = x.move(lx_speed, az_speed, d)
            print("Feedback: % 1d" % feedback)
        elif choice == 1:
            interaction: int = 0
            while interaction < 3:
                interaction = input("Type 0 to move, 1 to rotate left, 2 to rotate right, 3 to stop: ")
                if interaction == 0:
                    feedback = x.move(linear_speed=1)
                    print("Feedback: % 1d" % feedback)
                elif interaction == 1:
                    feedback = x.move(linear_speed=0.1, angular_speed=1)
                    print("Feedback: % 1d" % feedback)
                elif interaction == 2:
                    feedback = x.move(linear_speed=0.1, angular_speed=-1)
                    print("Feedback: % 1d" % feedback)

    except rospy.ROSInterruptException:
        pass
