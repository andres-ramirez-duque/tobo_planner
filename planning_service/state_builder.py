import os
from fondparser import parser, predicate, formula
import state_progressor
from fondparser.predicate import Predicate
import oyaml as yaml

LOG=False

class StateValueFrame(object):
  def __init__(self, name, default_v, values, source):
    self.name = name
    self.default_v=default_v
    self.values = values
    self.source=source
  def __str__(self):
    return self.name + "={" + ",".join(self.values())+"}"
class MultiValueFrame(StateValueFrame):
  def __init__(self,name,default_v,values, source):
    super(MultiValueFrame, self).__init__(name,default_v,values, source)
class BooleanValueFrame(StateValueFrame):
  def __init__(self,name, default_v,source):
    super(BooleanValueFrame, self).__init__(name,default_v,(False,True), source)

class VariableValuator(object):
  def get_bool_var_value(self, v):
    pass
  def get_multi_var_value(self, v):
    pass
class StateManagerProxy(VariableValuator):
  def get_last_executed_action(self):
    pass
  def get_previous_action(self):
    pass
  def set_previous_action(self, a):
    pass
class DummyStateManagerProxy(StateManagerProxy):
  def __init__(self, frames):
    self._dsm = __import__('dummy_state_manager')
    self.frames=dict(map(lambda f: (f.name,f),frames))
  def get_bool_var_value(self, v):
    return self.get_var_value(v)
  def get_multi_var_value(self, v):
    return self.get_var_value(v)
  def get_var_value(self, v):
    frame = self.frames[v]
    return self._dsm.request_feature_value(frame.name, frame.values)
  def get_last_executed_action(self):
    return self._dsm.request_value("which was the last executed action?")
  def get_previous_action(self):
    return self._dsm.request_value("which was the previous action?")
class RosStateManagerProxy(StateManagerProxy):
  def __init__(self):
    self.rospy = __import__('rospy')
    params = self.rospy.get_param("/parameters")
    if "boolean_vars" in params:
      self.bool_vars = params["boolean_vars"]
    if "multi_vars" in params:
      self.multi_vars = params["multi_vars"]
    self.executed_action = params["last_executed_action"]
    self.previous_action = params["previous_action"]
  def get_bool_var_value(self, v):
    if v in self.bool_vars:
      return self.bool_vars[v]
    print ("WARNING: missing key in state manager dictionary: " + str(v))
  def get_multi_var_value(self, v):
    if v in self.multi_vars:
      return self.multi_vars[v]
    print ("WARNING: missing key in state manager dictionary: " + str(v))
  def get_last_executed_action(self):
    return self.executed_action
  def get_previous_action(self):
    return self.previous_action
  def set_previous_action(self, a):
    self.rospy.set_param("/parameters/previous_action", a)

class InternalStateManager(VariableValuator):
  def __init__(self, next_states, P, frames):
    self.next_states=next_states
    self._P=P
    self.frames=dict(map(lambda f: (f.name,f),frames))
  def get_bool_var_value(self, v):
    return self.find_in_state(v)
  def get_multi_var_value(self, v):
    frame = self.frames[v]
    for e in frame.values:
      if self.find_in_state(e):
        return e
  def find_in_state(self, v):
    s = self.select_next_state()
    bits = v.split(" ")
    for e in s:
      if e.name == bits[0] and bits[1:] == map(lambda (a,t): a, e.ground_args):
        return True
    return False
  def select_next_state(self):
    return self.next_states[0]

class InitialisingStateManager(VariableValuator):
  def __init__(self, frames):
    self.frames=dict(map(lambda f: (f.name,f),frames))
  def get_bool_var_value(self, v):
    frame = self.frames[v]
    return frame.default_v
  def get_multi_var_value(self, v):
    frame = self.frames[v]
    return frame.default_v

def parse_background_knowledge(domain_fn, background_knowledge_fn):
  return parser.Problem(domain_fn, background_knowledge_fn)

#def parse_state_frame(sffn):
#  with open(sffn, "r") as stream:
#    frame_dict=yaml.safe_load(stream)

#  frames=[]  
#  for (bool_k, bool_v) in frame_dict["boolean_vars"].items():
#    frames.append(BooleanValueFrame(bool_k))
#  mv_values_map = frame_dict["multi_vars_values"]
#  for (multi_k, multi_v) in frame_dict["multi_vars"].items():
#    frames.append(MultiValueFrame(multi_k, mv_values_map[multi_k]))
#  return frames
  
def parse_state_frame(sffn):
  with open(sffn, "r") as stream:
    frame_dict=yaml.safe_load(stream)

  frames=[]  
  for source in ("parameters","interaction_manager","planner"):
    if not source in frame_dict : continue
    
    source_vars = frame_dict[source]
    if source_vars == None: continue
    
    if "boolean_vars" in source_vars: #if not source_vars["boolean_vars"]== None:
      for (bool_k, bool_v) in source_vars["boolean_vars"].items():
        frames.append(BooleanValueFrame(bool_k, str(bool_v).lower()=="true", source))

    if "multi_vars" in source_vars:#if not source_vars["multi_vars"]== None:
      mv_values_map = source_vars["multi_vars_values"]
      for (multi_k, multi_v) in source_vars["multi_vars"].items():
        frames.append(MultiValueFrame(multi_k, multi_v, mv_values_map[multi_k], source))
  return frames

def make_proposition(P, value):
  content = value.split(" ")
  args = map(lambda o: tuple([o]+ list(P.obj_to_type[o])), content[1:])
  p = predicate.Predicate(content[0], args)
  return formula.Primitive(p)

def add_boolean_value_to_state(P, name, value):
  if str(value).lower()== "true" :
    P.init.args.append(make_proposition(P, name))
def add_multi_value_to_state(P, value):
  P.init.args.append(make_proposition(P, value))

def complete_state_description(P, state_frames, state_manager, internal_state_progressor):
  proposition_source_map={"parameters": state_manager,
                          "interaction_manager": state_manager,
                          "planner":internal_state_progressor}
  for frame in state_frames :
    value_getter=proposition_source_map[frame.source]
    if isinstance (frame, BooleanValueFrame):
      boolean_getter = getattr(value_getter, "get_bool_var_value")
      add_boolean_value_to_state(P, frame.name, boolean_getter(frame.name))
    elif isinstance (frame, MultiValueFrame):
      multi_getter = getattr(value_getter, "get_multi_var_value")
      add_multi_value_to_state(P, multi_getter(frame.name))
    else :
      print ("WARNING: unknown variable type ", frame.__class__)

def get_state_manager(is_ros, frames):
  sm=None
  if is_ros:
    sm=RosStateManagerProxy()
  else:
    sm=DummyStateManagerProxy(frames)
  return sm

def get_state_progressor(next_states, P, frames):
  return InternalStateManager(next_states, P, frames)
def still_on_last_action(a1, a2):
  return a1==a2


def make_current_scenario(domain_fn, background_knowledge_fn, state_frame_fn, output_scenario_fn, is_ros):
  P = parse_background_knowledge(domain_fn, background_knowledge_fn)
  state_frames = parse_state_frame(state_frame_fn)
  state_manager = get_state_manager(is_ros, state_frames)
  executor_a_complete = state_manager.get_last_executed_action()
  previous_a=state_manager.get_previous_action()
  if executor_a_complete == '':
    internal_state_progressor = InitialisingStateManager(state_frames)
  else:
    if still_on_last_action(executor_a_complete, previous_a):
      a = ''
    else:
      a = executor_a_complete
      state_manager.set_previous_action(a)
    next_states = state_progressor.progress_state(domain_fn, output_scenario_fn, a)
    internal_state_progressor = get_state_progressor(next_states, P, state_frames)
  complete_state_description(P, state_frames, state_manager, internal_state_progressor)
  return P

def _export_problem (P, fp, sp="  ", is_costed=False):
  """Write the problem PDDL to given file."""
  fp.write ("(define" + "\n")
  fp.write (sp + "(problem %s)%s" % (P.problem_name, "\n"))
  fp.write (sp + "(:domain %s)%s" % (P.domain_name, "\n"))

  # objects
  o = []
  o.append (sp + "(:objects")
  for obj in P.objects:
    if P.obj_to_type[obj] == Predicate.OBJECT:
      o.append (sp + sp + obj)
    else:
      #TODO may not be correct
      t = list (P.obj_to_type [obj]) [0]
      o.append (sp + sp + "%s - %s" % (obj, t))
  o.append (sp + ")")
  fp.write ("\n".join(o) + "\n")

  # init
  o = []
  o.append (sp + "(:init")
  if is_costed:
    o.append(2*sp + "(= (total-cost) 0)")
  for f in P.init.args:
    o.append (f.export (2, sp, True))
  o.append (sp + ")") # close init
  fp.write ("\n".join(o) + "\n")

  # goal
  o = []
  o.append (sp + "(:goal(and")
  for p in P.goal.args:
    o.append (p.export (2, sp, True))
  o.append (sp + "))") # close goal
  fp.write ("\n".join (o) + "\n")
  if is_costed:
    fp.write ("(:metric minimize (total-cost))\n") # metric
  fp.write (")") # close define

def output_scenario(P, output_scenario_fn, is_costed):
  _export_problem(P, open(output_scenario_fn,'w'), is_costed=is_costed)

def build_scenario(domain_fn, background_knowledge_fn, state_frame_fn, output_scenario_fn, is_ros=False, is_costed=False):
  P = make_current_scenario(domain_fn, background_knowledge_fn, state_frame_fn, output_scenario_fn, is_ros)
  output_scenario(P, output_scenario_fn, is_costed)

if __name__ == '__main__':
  domain_fn, background_knowledge_fn, state_frame_fn, output_scenario_fn = os.sys.argv[1:]
  build_scenario(domain_fn, background_knowledge_fn, state_frame_fn, output_scenario_fn)


