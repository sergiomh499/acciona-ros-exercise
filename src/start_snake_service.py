#!/usr/bin/env python
import rospy
import roslaunch
import time
import random
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, Spawn
from practica_acciona.srv import StartTurtlesimSnake, StartTurtlesimSnakeResponse

def handler_start_turtlesim_snake(req):
    """
    Handler of the Start Turtlesim Snake service
    """
    print("\n-> Starting Turtlesim Snake...")
    rospy.set_param('/start_turtlesim_snake', True)
    rospy.set_param('/x_ini', req.x)
    rospy.set_param('/y_ini', req.y)
    rospy.set_param('/theta_ini', req.theta)
    rospy.set_param('/camera_control', req.camera_control)
    return StartTurtlesimSnakeResponse("Have a good game! :)")


def start_turtlesim_server():
    """
    Start Turtlesim Snake Server to the start service
    """
    rospy.init_node('start_turtlesim_snake_server', anonymous=True)
    service = rospy.Service('start_turtlesim_snake', StartTurtlesimSnake, handler_start_turtlesim_snake)
    print("\n- Established Start Turtlesim Snake service...")


if __name__ == '__main__':
    # initializate start program parameter
    rospy.set_param('/start_turtlesim_snake', False)
    # initializate start turtle sim service
    start_turtlesim_server()

    while not rospy.is_shutdown():
        # check if start the program
        if rospy.get_param('/start_turtlesim_snake'):
            # initialization
            turtle_number = 1
            turtle_name = 'turtle' + str(turtle_number)
            rospy.set_param('/last_turtle', 'turtle1')
            start_time = time.time()
            pose = Pose()

            # initialization of the roslaunch server
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()

            # launch turtlesim node
            node = roslaunch.core.Node('turtlesim', 'turtlesim_node')
            launch.launch(node)
            
            # type of control selection
            control = rospy.get_param('/control')
            if control == "camera":
                # launch camera control
                node = roslaunch.core.Node('practica_acciona', 'camera_control.py')
            elif control == "web":
                # launch web control
                node = roslaunch.core.Node('practica_acciona', 'web_control.py')
            else: # default
                # launch turtlesim teleop key
                node = roslaunch.core.Node('turtlesim', 'turtle_teleop_key', 
                                        output="screen", launch_prefix="xterm -e")
            launch.launch(node)

            # launch turtle broadcaster node
            node = roslaunch.core.Node('practica_acciona', 'turtle_broadcaster.py',
                                        args="_nr:=%s" % str(turtle_number))
            launch.launch(node)

            # teleport the initial turtle to indicated location
            rospy.wait_for_service('/turtle1/teleport_absolute')
            teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
            teleport(rospy.get_param('/x_ini'),
                    rospy.get_param('/y_ini'),
                    rospy.get_param('/theta_ini'))


            while not rospy.is_shutdown():
                if time.time() - start_time > random.uniform(5, 8):
                    # update variables
                    turtle_number += 1
                    turtle_name = 'turtle' + str(turtle_number)

                    # spawn new turtle
                    print("\n--> Spawn turtle" + str(turtle_number))
                    rospy.wait_for_service('spawn')
                    spawner = rospy.ServiceProxy('spawn', Spawn)
                    pose.x = random.uniform(1, 9)
                    pose.y = random.uniform(1, 9)
                    pose.theta = random.uniform(-3, 3)
                    spawner(pose.x, pose.y, pose.theta, turtle_name)
                    
                    # launch turtle broadcaster node
                    node = roslaunch.core.Node('practica_acciona', 'turtle_broadcaster.py',
                                                args="_nr:=%s" % str(turtle_number))
                    launch.launch(node)
                    # launch turtle controller node
                    node = roslaunch.core.Node('practica_acciona', 'turtle_controller.py',
                                                args="_nr:=%s" % str(turtle_number))
                    launch.launch(node)
                    
                    # reset the start time
                    start_time = time.time()
            
