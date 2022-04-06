#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import String
from tobo_planner.srv import PlannerMode,PlannerModeResponse

import getanaction as actionselector

def handle_get_an_action(req):
    pub = rospy.Publisher('next_action', String, queue_size=10)

    next_action = actionselector.get_an_action(actionselector.ros_parameter_service(), is_ros=True)

    pub.publish(next_action)
    return PlannerModeResponse(1)

def remove_if_exists(fn):
  if os.path.exists(fn):
    os.remove(fn)
    
def get_an_action_service():
    rospy.init_node('get_an_action_service')
    s = rospy.Service('get_an_action', PlannerMode, handle_get_an_action)
    solution_fn= rospy.get_param('solution_fn', 'aplan.out')
    scenario_fn = rospy.get_param('scenario_fn', 'pout.pddl')
    remove_if_exists(scenario_fn)
    remove_if_exists(solution_fn)
    print("Ready to get an action.")
    rospy.spin()


if __name__ == '__main__':
    get_an_action_service()
  

