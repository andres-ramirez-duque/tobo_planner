#!/usr/bin/env python

import os, sys, subprocess


############################################################################################################
### defaults 
############################################################################################################

deadend_detection=True
use_local_search=True
keep_files=False


############################################################################################################
### local imports and ugly path stuff
### parameters 
############################################################################################################

PRP_LOCATIONS=["/home/al/ukcan/planner-for-relevant-policies",
               "/home/toboraspuk/usrlib/prp"]
for loc in PRP_LOCATIONS:
  if os.path.exists(loc):
    PRP_ROOT=loc
    break
else:
  print "ERROR: Cannot locate PRP in getanaction.py - add to PRP_LOCATIONS"
  sys.exit(1)

sys.path.append(PRP_ROOT+"/prp-scripts")
import state_builder, make_prp_runner, planner


############################################################################################################
### parameters 
############################################################################################################

class parameter_service(object):
  def get_param_value(self, param, default):
    pass
class ros_parameter_service(parameter_service):
  def __init__(self):
    self.rospy = __import__('rospy')
  def get_param_value(self, param, default):
    return self.rospy.get_param(param, default)
class dummy_parameter_service(parameter_service):
  def __init__(self, fn):
    self.d = dict(map(lambda x: x.split(":"), filter(lambda x: ":" in x, map(lambda x: x.strip(), open(fn)))))
  def get_param_value(self, param, default):
    if param in self.d:
      return self.d[param]
    return default

############################################################################################################
### get an action functions
############################################################################################################

def build_scenario(domain_fn, background_knowledge_fn, state_frame_fn, scenario_fn, is_ros, is_costed):
  state_builder.build_scenario(domain_fn, background_knowledge_fn, state_frame_fn, scenario_fn, is_ros, is_costed)

def get_next_action(domain_fn, scenario_fn, solution_fn, prp_root, cmplan_abs_path, is_costed):
  make_prp_runner.make_prp_runner(prp_root, solution_fn, deadend_detection, cmplan_abs_path, is_costed, use_local_search)
  run_a_command( ["chmod", "u+x", cmplan_abs_path])
  return planner.get_next_action(domain_fn, scenario_fn, cmplan_abs_path, solution_fn, is_costed)

def run_a_command(command_args):
  print " ".join(map(lambda x: str(x), command_args))
  exit_c = subprocess.call(command_args)
  return exit_c

def remove_if_exists(fn):
  if os.path.exists(fn):
    os.remove(fn)

def cleanup(prp_root):
  if not keep_files:
    run_a_command([prp_root+"/src/cleanup"])
    remove_if_exists("policy.out")
    remove_if_exists("human_policy.out")
    remove_if_exists("graph.dot")
    remove_if_exists("action.map")
    remove_if_exists("graph.png")
    remove_if_exists("unhandled.states")
    remove_if_exists("policy.fsap")
    
#def link_with_PRP(prp_root):
#  sys.path.append(prp_root+"/prp-scripts")
#  import state_builder, make_prp_runner, planner

def get_an_action(parameter_service, is_ros=False):
  is_costed = str(parameter_service.get_param_value('costed_domain', 'false')).lower()=="true"
  domain_fn = parameter_service.get_param_value('domain_fn', 'model0.2/domain_plan.pddl')
  #costed_domain_fn = parameter_service.get_param_value('costed_domain_fn', None)
  background_knowledge_fn = parameter_service.get_param_value('background_knowledge_fn', 'model0.2/scenario_background_knowledge.pddl')
  state_frame_fn = parameter_service.get_param_value('state_frame_fn', 'model0.2/state_frames_scenario.txt')
  scenario_fn = parameter_service.get_param_value('scenario_fn', 'pout.pddl')
  solution_fn= parameter_service.get_param_value('solution_fn', 'aplan.out')
  prp_root = parameter_service.get_param_value('PRP_ROOT', '/home/al/ukcan/planner-for-relevant-policies')
  # link_with_PRP(prp_root)
  cmplan_abs_path = parameter_service.get_param_value('CMPLAN_ABS_PATH','./cmplan')
    
  build_scenario(domain_fn, background_knowledge_fn, state_frame_fn, scenario_fn, is_ros, is_costed)
  next_action = str(get_next_action(domain_fn, scenario_fn, solution_fn, prp_root, cmplan_abs_path, is_costed))
  print ("THE ACTION: " + next_action)
  cleanup(prp_root)
  return next_action


############################################################################################################
### main
############################################################################################################

if __name__ == '__main__':
  parameter_service=dummy_parameter_service(os.sys.argv[1])
  get_an_action(parameter_service)
  
  
  
  
  
