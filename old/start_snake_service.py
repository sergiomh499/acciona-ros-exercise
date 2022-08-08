#!/usr/bin/env python

import rospy
import time
from turtlesim.srv import Spawn, TeleportAbsolute
from practica_acciona.srv import StartTurtlesimSnake, StartTurtlesimSnakeResponse
from practica_acciona.srv import TurtleController, TurtleBroadcaster

def start_turtlesim_snake_handler(req):
    print("Entry handler...")
    rospy.wait_for_service('/turtle1/teleport_absolute')
    print("Wait for service...")
    teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
    teleport(req.x, req.y, req.theta)
    # teleport(1, 1, 1)
    print("Teleport to " + str(req.x) + "," + str(req.y) + "," + str(req.theta))

    start_time = time.time()
    turtle_number = 1

    while not rospy.is_shutdown():
        if time.time() - start_time > 5.0:
            turtle_number += 1
            # call turtle_controller service to create and control a turtle
            rospy.wait_for_service('turtle_controller')
            controller = rospy.ServiceProxy('turtle_controller', TurtleController)
            controller(turtle_number)
            # call turtle_controller service to create and control a turtle
            rospy.wait_for_service('turtle_boradcaster')
            broadcaster = rospy.ServiceProxy('turtle_broadcaster', TurtleBroadcaster)
            broadcaster(turtle_number)

                
    return StartTurtlesimSnakeResponse('OK.')

if __name__ == '__main__':
    rospy.init_node('start_turtlesim_snake_respond')
    print("Initiate node...")
    service = rospy.Service('start_turtlesim_snake', StartTurtlesimSnake, start_turtlesim_snake_handler)
    print("Established service...")
    rospy.spin()