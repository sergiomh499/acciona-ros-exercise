#!/usr/bin/env python  
# import roslib
# roslib.load_manifest('learning_tf')
import rospy
import tf
import turtlesim.msg
from practica_acciona.srv import TurtleBroadcaster, TurtleBroadcasterResponse

def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")

def turtle_broadcaster_handler(req):
    rospy.init_node('broadcaster')
    rospy.Subscriber('/%s/pose' % req.turtle_name,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     req.turtle_name)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('turtle_broadcaster_response')
    print("Initiate node...")
    service = rospy.Service('turtle_broadcaster', TurtleBroadcaster, turtle_broadcaster_handler)
    print("Established service...")
    rospy.spin()
