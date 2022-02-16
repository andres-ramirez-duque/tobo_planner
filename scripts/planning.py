#!/usr/bin/env python

import sys
import parser
import random

import rospy
from std_msgs.msg import String
from tobo_planner.srv import *

ASK_FOR_SENSED_VALUES=True

def message(t, s):
  print t*"  " + "== " + str(s) + " =="


def request_sensed_value(t_id, options):
  message(2,"Choice: " + str(transition_d[t_id]))
  ordered_options = list(options)
  for i, option in enumerate(ordered_options):
    message(3, "Option [" + str(i) + "]: " + str(option))
  choice_ros=rospy.get_param("/choice_ros")  
  #choice = int(input ("    >>> Select sensed value from " + str(range(len(ordered_options)))+": "))
  rospy.wait_for_service('sensortransition')
  try:
      sensortransition = rospy.ServiceProxy('sensortransition', SensorTransition)
      ishighsensor = sensortransition(str(transition_d[t_id]))
  except rospy.ServiceException as e:
      print("Service call failed: %s"%e)
      
  return ordered_options[ishighsensor.sensor_state] #ishighsensor


def discover_next_node(n):
  if ASK_FOR_SENSED_VALUES:
    sv = request_sensed_value(n.transition_id, n.neighbours.keys())
  else:
    sv = random.choice((True,False))
  nn = n.neighbours[sv]
  return nn

if __name__ == '__main__':
    p_fn = rospy.get_param('/policy_path', '_policy.xml')
    t_fn = rospy.get_param('/transitions_path', '_transitions.xml')
    node_d,init = parser.parse_policy_xml(p_fn)
    transition_d = parser.parse_transitions_xml(t_fn)
    pub = rospy.Publisher('action', String, queue_size=10)
    rospy.init_node('planning', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    current_node = init
    message(0,"Entering Loop..")
    
    while not rospy.is_shutdown():
        while not current_node == -1:
            n = node_d[current_node]
            message(1,"Current node: [" + str(current_node) + "] " + str(n))
            message(2,"Current state: " + str(n.state_id))
  
            if isinstance(n, parser.ActionNode):
                message(2,"Transition: " + str(transition_d[n.transition_id]))
                pub.publish(str(transition_d[n.transition_id]))
                current_node = n.next_node
            else:
                current_node = discover_next_node(n)
            
            rate.sleep()
        
    message(0,"Exiting Loop.. with node: [" + str(current_node) + "]")
    
