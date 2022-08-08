#!/usr/bin/env python

import rospy
import roslaunch
import time
import random
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, Spawn
from practica_acciona.srv import StartTurtlesimSnake, StartTurtlesimSnakeResponse

# def dummy_function(): pass
# roslaunch.pmon._init_signal_handlers = dummy_function



def start_turtlesim_snake_handler(req):
    
    print("Starting Turtlesim Snake...")

    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)

    # #ros launch
    # print("--> INIT ROS LAUNCH ")
    # # rospy.init_node('init', anonymous=True)
    # cli_args = ['practica_acciona', 'simple_snake.launch']
    # roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    # launch_files = [roslaunch_file]
    # parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    # turtlesim_node
    node = roslaunch.core.Node('turtlesim', 'turtlesim_node')
    launch.launch(node)

    # turtle_teleop_key
    node = roslaunch.core.Node('turtlesim', 'turtle_teleop_key')
    launch.launch(node)

    # start_snake_service
    node = roslaunch.core.Node('practica_acciona', 'start_snake_service.py')
    launch.launch(node)

    # broadcaster turtle 1
    node = roslaunch.core.Node('practica_acciona', 'turtle_broadcaster.py', args="turtle1")
    launch.launch(node)

    rospy.wait_for_service('/turtle1/teleport_absolute')
    teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
    teleport(req.x, req.y, req.theta)
    print("Teleport to " + str(req.x) + "," + str(req.y) + "," + str(req.theta))

    
    start_time = time.time()
    turtle_number = 1
    pose = Pose()

    while not rospy.is_shutdown():
        if time.time() - start_time > 2.0:
            turtle_number += 1

            # spawn new turtle
            # define turtle properties
            turtle_number = turtle_number
            turtle_name = 'turtle' + str(turtle_number)
            # call turtlesim service to spawn turtle
            print("--> SPAWN TURTLE " + str(turtle_number))
            rospy.wait_for_service('spawn')
            spawner = rospy.ServiceProxy('spawn', Spawn)
            pose.x = random.uniform(1, 9)
            pose.y = random.uniform(1, 9)
            spawner(pose.x, pose.y, 0, turtle_name)


            # broadcaster turtle i
            node = roslaunch.core.Node('practica_acciona', 'turtle_broadcaster.py', args=turtle_name)
            launch.launch(node)
            
            # controller turtle i
            node = roslaunch.core.Node('practica_acciona', 'turtle_controller.py', args=turtle_name)
            launch.launch(node)
            
            # #ros launch new turtle
            # print("--> ROS LAUNCH NEW TURTLE " + str(turtle_number))
            # cli_args = ['practica_acciona', 'turtle.launch', 'nr:=' + str(turtle_number)]
            # roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
            # roslaunch_args = cli_args[2:]
            # launch_files = [(roslaunch_file, roslaunch_args)]
            # parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

            # parent.start()

            # start_time = time.time()

            # try:
            #     parent.spin()
            # finally:
            #     parent.shutdown()
                
    return StartTurtlesimSnakeResponse('OK.')

if __name__ == '__main__':
    rospy.init_node('start_turtlesim_snake_respond', anonymous=True)
    print("----> Initiate Start Turtlesim Snake  node...")
    service = rospy.Service('start_turtlesim_snake', StartTurtlesimSnake, start_turtlesim_snake_handler)
    print("----> Established Start Turtlesim Snake service...")
    rospy.spin()