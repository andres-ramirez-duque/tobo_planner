#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from tobo_planner.msg import action_chain
from diagnostic_msgs.msg import KeyValue
from diagnostic_msgs.msg import DiagnosticStatus
from tobo_planner.srv import PlannerMode,PlannerModeResponse

import getanaction as actionselector

def handle_get_an_action(req):
    pub = rospy.Publisher('next_action', action_chain, queue_size=10)
    ac_msg = action_chain()
    ac_msg.caller_id = 0
    ac_msg.plan_step = req.plan_step
    
    plan_output = actionselector.get_an_action(actionselector.ros_parameter_service(), is_ros=True)
    action_bits = plan_output.split("_")

    ac_msg.action_type = action_bits[0]
    ac_msg.parameters = action_bits[1:]
    
    temp_status = DiagnosticStatus()
    temp_status.name = "Action Execution Status"
    temp_status.hardware_id = "raspberry_pi4b"
    # temp_status.values.append(KeyValue(key="Planner_status", value="ok"))
    temp_status.level = temp_status.OK
    ac_msg.execution_status = temp_status
    
    pub.publish(ac_msg)
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
  

