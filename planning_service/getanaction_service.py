#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from tobo_planner.srv import PlannerMode,PlannerModeResponse

import getanaction as actionselector

def handle_get_an_action(req):
    pub = rospy.Publisher('next_action', Header, queue_size=10)
    h = Header()
    h.stamp = rospy.Time.now()
    next_action = actionselector.get_an_action(actionselector.ros_parameter_service(), is_ros=True)
    h.frame_id = next_action
    pub.publish(h)
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
  

