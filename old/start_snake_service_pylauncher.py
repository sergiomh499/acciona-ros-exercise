#!/usr/bin/env python
# https://medium.com/@shilpajbhalerao/ros-pylauncher-9ac50951e230
import rospy
import roslaunch
import time
import random
import math
import tf
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, Spawn, Kill, SetPen
from practica_acciona.srv import StartTurtlesimSnake, StartTurtlesimSnakeResponse

distance_tolerance = 0.5

# def dummy_function(): pass
# roslaunch.pmon._init_signal_handlers = dummy_function

class Snake:
    """
    Class to define Snake in the turtle-sim
    """
    def __init__(self):
        self.Turtles = []


    def get_last_turtlename(self):
        return self.Turtles[-1].get_name()

    def add_turtle(self, turtle):
        self.Turtles.Append(turtle)

class Turtle:
    """
    Class to control Turtles in the turtle-sim
    """
    def __init__(self, i):
        self.name = 'turtle' + str(i)

    def __repr__(self):
        return 'Turtle Name: {}'.format(self.name)

    def get_name(self):
        """
        Method to extract name of the turtle instance
        """
        return self.name

    def spawn(self, x_pos, y_pos, theta):
        """
        Function to spawn turtles in the Turtle-sim
        :param x_pos: x-position with respect to origin at bottom-left
        :type x_pos: float
        :param y_pos: y-position with respect to origin at bottom-left
        :type y_pos: float
        :param theta: orientation with respect to x-axis
        :type theta: float between [0 to 3] OR [0 to -3]
        """
        try:
            serv = rospy.ServiceProxy('/spawn', Spawn)
            serv(x_pos, y_pos, theta, self.name)
        except rospy.ServiceException as error:
            rospy.loginfo("Service execution failed: %s" + str(error))

    def set_pen(self, flag=True):
        """
        Function to sketch the turtle movements
        :param flag: To turn sketching pen - ON[True]/OFF[False]
        :type flag: bool
        """
        try:
            if not flag:
                set_serv = rospy.ServiceProxy('/' + self.name + '/set_pen', SetPen)
                set_serv(0, 0, 0, 0, 1)
            elif flag:
                set_serv = rospy.ServiceProxy('/' + self.name + '/set_pen', SetPen)
                set_serv(255, 255, 255, 2, 0)
        except rospy.ServiceException as error:
            rospy.loginfo("Service execution failed: %s" + str(error))

    def teleport(self, x_pos, y_pos, theta):
        """
        Function to teleport the turtle
        :param x_pos: x-position with respect to origin at bottom-left
        :type x_pos: float
        :param y_pos: y-position with respect to origin at bottom-left
        :type y_pos: float
        :param theta: orientation with respect to x-axis
        :type theta: float between [0 to 3] OR [0 to -3]
        """
        try:
            serv = rospy.ServiceProxy('/' + self.name + '/teleport_absolute', TeleportAbsolute)
            serv(x_pos, y_pos, theta)
        except rospy.ServiceException as error:
            rospy.loginfo("Service execution failed: %s" + str(error))

    def kill_turtle(self):
        """
        Function to remove the turtle from Turtle-sim
        """
        try:
            serv = rospy.ServiceProxy('/kill', Kill)
            serv(self.name)
        except rospy.ServiceException as error:
            rospy.loginfo("Service execution failed: %s" + str(error))


class Instance:
    """
    Class for multiple instance launch
    """
    def __init__(self, value):
        # Initialize a node
        rospy.init_node('turtle', anonymous=False)

        # Log node status
        rospy.loginfo("Node Initialized: Turtle"+str(value))

        # Private variable for the number of instance
        self._instance_number = value

        # Broadcaster and listerner
        self.broadcast = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # Variables for position and orientation of turtles
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0

        # Rate
        self.rate = rospy.Rate(10.0)

        # Object of a Turtle class to interact with turtles
        self.turtle = None

        # Define if it is follower and who is the leader turtle
        self.follower = False
        self.leader_turtle = None

        # Spawn turtles randomly
        self.random_spawn()

        # Initiate topic for instance and publisher to publish data
        topic_name = '/Node' + str(self._instance_number) + '/topic' + str(self._instance_number)
        self.pub = rospy.Publisher(topic_name, Int64, queue_size=1)

        while not rospy.is_shutdown():
            # Broadcast tf
            self.dynamic_frame()

            # Publish data
            self.pub.publish(self._instance_number)

            if not self.follower:
                (trans,rot) = listener.lookupTransform('/' + self.turtle.get_name(), '/turtle1', rospy.Time(0))
                self.check_collisions(trans)
            else:
                (trans,rot) = listener.lookupTransform('/' + self.turtle.get_name(), '/' + self.leader_turtle, rospy.Time(0))
                self.follow_turtle(trans)

            self.rate.sleep()

    def random_spawn(self):
        """
        Method to spawn only a single instance of a turtle
        """
        self.turtle = Turtle(self._instance_number)

        self.rand_pos()

        self.turtle.spawn(self.x_pos, self.y_pos, self.theta)
        print(self.turtle.get_name())

    def rand_pos(self):
        """
        Method to set a random position and orientation of a turtle in a turtlesim
        """
        self.x_pos = random.randint(0, 11)
        self.y_pos = random.randint(0, 11)
        self.theta = random.random()

    def dynamic_frame(self):
        """
        Method to broadcast the dynamic transform
        """
        time_now = rospy.Time.now().to_sec() * math.pi
        self.broadcast.sendTransform((2.0 * math.sin(time_now), 2.0 * math.cos(time_now), 0.0),
                                     (0.0, 0.0, 0.0, 1.0),
                                     rospy.Time.now(),
                                     "turtle" + str(self._instance_number),
                                     "world")

    def check_collision(self, trans):
        if math.sqrt(trans[0] ** 2 + trans[1] ** 2) < distance_tolerance:
            collision = True
            self.follower = True
        else:
            collision = False
        return collision

    def follow_turtle(self, trans):
        if math.sqrt(trans[0] ** 2 + trans[1] ** 2) >= distance_tolerance:
            angular = 6 * math.atan2(trans[1], trans[0])
            linear = 2 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        else:
            angular = 0
            linear = 0
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.vel_pub.publish(cmd)


def start_turtlesim_snake_handler(req):
    
    start_time = time.time()
    turtle_number = 1
    
    print("Creating snake...")
    snake = Snake()
    print("Adding first turtle to snake...")
    turtle = Instance(turtle_number)
    snake.add_turtle(turtle)

    while not rospy.is_shutdown():
        if time.time() - start_time > 2.0:
            turtle_number += 1

            turtle = Instance(turtle_number)
            snake.add_turtle(turtle)

            start_time = time.time()
                
    return StartTurtlesimSnakeResponse('OK.')

if __name__ == '__main__':
    rospy.init_node('start_turtlesim_snake_respond')
    print("----> Initiate Start Turtlesim Snake  node...")
    service = rospy.Service('start_turtlesim_snake', StartTurtlesimSnake, start_turtlesim_snake_handler)
    print("----> Established Start Turtlesim Snake service...")
    rospy.spin()