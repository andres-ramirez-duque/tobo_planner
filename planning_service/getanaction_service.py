#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from tobo_planner.srv import PlannerMode,PlannerModeResponse

import getanaction as actionselector

def handle_get_an_action(req):
    pub = rospy.Publisher('next_action', String, queue_size=10)

    next_action = actionselector.getanaction(actionselector.ros_parameter_service(), is_ros=True)

    pub.publish(next_action)
    return PlannerModeResponse(1)

def get_an_action_service():
    rospy.init_node('get_an_action_service')
    s = rospy.Service('get_an_action', PlannerMode, handle_get_an_action)
    print("Ready to get an action.")
    rospy.spin()


if __name__ == '__main__':
    get_an_action_service()
  

