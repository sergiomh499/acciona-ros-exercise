#!/usr/bin/env python  
import rospy
import math
import tf
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

linear_distance_tolerance = 0.5

def last_turtle_callback(data):
    return data.data

class Turtle:
    def __init__(self, turtle_number):
        """
        Initialization of the turtle
        
        args:
            turtle_number: turtle identification number
        """
        # initialize variables
        self.turtle_number = turtle_number
        self.turtle_name = 'turtle' + str(self.turtle_number)
        self.pose = Pose()
        self.follower = False # is not a follower turtle
        # setting the turtle velocity publisher
        self.vel_pub = rospy.Publisher(self.turtle_name + '/cmd_vel', Twist, queue_size=1) 

    def check_collision(self, trans):
        """
        Check the collision wby the translation between other turtle

        args:
            trans: translation
        """
        distance_linear = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        if distance_linear < linear_distance_tolerance:
            collision = True
            self.follower = True
            self.last_turtle = rospy.get_param('/last_turtle')
            rospy.set_param('/last_turtle', self.turtle_name)
        else:
            collision = False
        return collision

    def follow_turtle(self, trans):
        """
        Follow the turtle indicated by the translation between other turtle

        args:
            trans: translation
        """
        distance_linear = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        distance_angular = math.atan2(trans[1], trans[0])
        if distance_linear >= linear_distance_tolerance:
            vel_angular = 6.0 * distance_angular
            vel_linear = 2.0 * distance_linear
        else:
            vel_angular = 0.0
            vel_linear = 0.0
        cmd = Twist()
        cmd.linear.x = vel_linear
        cmd.angular.z = vel_angular
        self.vel_pub.publish(cmd)


if __name__ == '__main__':
    # init turtle controller node
    rospy.init_node('controller')
    # initializate variables
    turtle_number = rospy.get_param('~nr')
    turtle = Turtle(turtle_number)
    rate = rospy.Rate(20.0)
    # initizalizate tf listener
    listener = tf.TransformListener() 

    while not rospy.is_shutdown():
        if not turtle.follower:
            try:
                # if not follower, check collision with turtle 1
                (trans, rot) = listener.lookupTransform('/turtle' + str(turtle_number), '/turtle1', rospy.Time(0))
                turtle.check_collision(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        else:
            try:
                # if follower, follow the last turtle on the snake
                (trans, rot) = listener.lookupTransform('/turtle' + str(turtle_number), '/' + turtle.last_turtle, rospy.Time(0))
                turtle.follow_turtle(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue   
        rate.sleep()
        
        