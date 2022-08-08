#!/usr/bin/env python  
import rospy
import tf
from turtlesim.msg import Pose

def handle_turtle_pose(msg, turtlename):
    """
    Handler of the turtle pose subscription

    args: 
        turtlename: turtle identification name
    """
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")


if __name__ == '__main__':
    # init the turtle broadcaster node
    rospy.init_node('broadcaster')
    # initializarion of variables
    turtle_number = rospy.get_param('~nr')
    turtle_name = 'turtle' + str(turtle_number)
    # subscribe to turtle pose
    rospy.Subscriber('/%s/pose' % turtle_name,
                     Pose,
                     handle_turtle_pose,
                     turtle_name)
    rospy.spin()
