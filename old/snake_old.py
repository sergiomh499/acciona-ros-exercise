#!/usr/bin/env python
import rospy
import math
import random
import tf
import turtlesim.srv
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

# constants
distance_tolerance = 1.0

class Turtle:
    def __init__(self):
        # initialize variables
        self.pose = Pose()
        self.pose.x = random.uniform(1, 9)
        self.pose.y = random.uniform(1, 9)
        self.follower = False # is not a follower turtle
        self.rate = rospy.Rate(100.0)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return math.sqrt(pow((goal_pose.x - self.pose.x), 2) +
                            pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=0.1):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=0.6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def spawn(self, turtle_number):
        # define turtle propierties
        self.turtle_number = turtle_number
        self.turtle_name = 'turtle' + str(self.turtle_number)

        # call turtlesim service to spawn turtle
        rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        spawner(self.pose.x, self.pose.y, 0, self.turtle_name)

        print("NEW TURTLE SPAWNED! (" + str(self.pose.x) + ", " + str(self.pose.y) + ")")
        print("")

        # define turtle velocity publisher
        self.velocity_publisher = rospy.Publisher(self.turtle_name + '/cmd_vel',
                                                  Twist, queue_size=10)

        # a subscriber to the topic '/pose'
        self.pose_subscriber = rospy.Subscriber(self.turtle_name + '/pose',
                                                Pose, self.update_pose)

    def check_collision(self, leader_pose):
        if self.euclidean_distance(leader_pose) < distance_tolerance:
            print(str(self.turtle_name) + " euclidean distance to leader: " + str(self.euclidean_distance(leader_pose)))
            collision = True
            self.follower = True
        else:
            collision = False
        return collision
        
    def follow_next_turtle(self, next_turtle_pose):
        vel_msg = Twist()
        print("Distance of " + self.turtle_name + " with next turtle: " + 
                        str(self.euclidean_distance(next_turtle_pose)))

        while self.euclidean_distance(next_turtle_pose) >= distance_tolerance:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(next_turtle_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(next_turtle_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':

    # initialization
    turtles = []

    # initialization of turtle followers node
    rospy.init_node('turtle_followers')

    # create leader turtle
    turtles.append(Turtle())

    # subscribe to leader turtle pose
    leader_pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, turtles[0].update_pose)

    # spawn first turtle
    turtles.append(Turtle())
    turtles[1].spawn(str(len(turtles)))

    while not rospy.is_shutdown():
        # STATE 1: Check collision with the new turtle and if collision create a new turtle
        if turtles[-1].check_collision(turtles[0].pose):
            turtles.append(Turtle())
            turtles[-1].spawn(str(len(turtles)))

        # turtles start to follow the next one
        for idx, turtle in enumerate(turtles):
            if turtle.follower:
                # print(str(turtle.turtle_name) + " is following next turtle...")
                turtle.follow_next_turtle(turtles[idx-1].pose)
