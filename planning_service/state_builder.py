import os
from fondparser import parser, predicate, formula
import state_progressor
import oyaml as yaml

LOG=False

class StateValueFrame(object):
  def __init__(self, name, values, source):
    self.name = name
    self.values = values
    self.source=source
  def __str__(self):
    return self.name + "={" + ",".join(self.values())+"}"
class MultiValueFrame(StateValueFrame):
  def __init__(self,name,values, source):
    super(MultiValueFrame, self).__init__(name,values, source)
class BooleanValueFrame(StateValueFrame):
  def __init__(self,name, source):
    super(BooleanValueFrame, self).__init__(name,(False,True), source)

class VariableValuator(object):
  def get_bool_var_value(self, v):
    pass
  def get_multi_var_value(self, v):
    pass
class StateManagerProxy(VariableValuator):
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
class RosStateManagerProxy(StateManagerProxy):
  def __init__(self):
    rospy = __import__('rospy')
    params = rospy.get_param("/parameters")
    self.bool_vars = params["boolean_vars"]
    self.multi_vars = params["multi_vars"]
  def get_bool_var_value(self, v):
    if v in self.bool_vars:
      return self.bool_vars[v]
    print ("WARNING: missing key in state manager dictionary: " + str(v))
  def get_multi_var_value(self, v):
    if v in self.multi_vars:
      return self.multi_vars[v]
    print ("WARNING: missing key in state manager dictionary: " + str(v))

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

#print ("WARNING: missing key in state manager dictionary: " + str(frame.name))

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
    source_vars = frame_dict[source]
    if source_vars == None: continue
    
    if not source_vars["boolean_vars"]==None:
      for (bool_k, bool_v) in source_vars["boolean_vars"].items():
        frames.append(BooleanValueFrame(bool_k, source))

    if not source_vars["multi_vars"]==None:
      mv_values_map = source_vars["multi_vars_values"]
      for (multi_k, multi_v) in source_vars["multi_vars"].items():
        frames.append(MultiValueFrame(multi_k, mv_values_map[multi_k], source))
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

def make_current_scenario(domain_fn, background_knowledge_fn, state_frame_fn, is_ros):
  P = parse_background_knowledge(domain_fn, background_knowledge_fn)
  #import sys, domain_projection as projector
  #for a in P.actions: print projector.project_action(a, ['naustep','procedurestep','anxteststep']).export()
  #sys.exit()
  #executor_a_complete="doactivity1 intro intronau introstep".strip().replace(' ', '_')
  #previous_a=''.strip().replace(' ', '_')
  executor_a_complete="doactivity1_song2_reward_debrief".strip().replace(' ', '_')
  previous_a='progressprocstep2_procedure_debrief'.strip().replace(' ', '_')
  if still_on_last_action(executor_a_complete, previous_a):
    a = ''
  else:
    a = executor_a_complete
  next_states = state_progressor.progress_state(domain_fn, "pout.pddl", a)
  state_frames = parse_state_frame(state_frame_fn)
  state_manager = get_state_manager(is_ros, state_frames)
  internal_state_progressor = get_state_progressor(next_states, P, state_frames)
  complete_state_description(P, state_frames, state_manager, internal_state_progressor)
  return P

def output_scenario(P, output_scenario_fn):
  P._export_problem(open(output_scenario_fn,'w'))

def build_scenario(domain_fn, background_knowledge_fn, state_frame_fn, output_scenario_fn, is_ros=False):
  p = make_current_scenario(domain_fn, background_knowledge_fn, state_frame_fn, is_ros)
  output_scenario(p, output_scenario_fn)

if __name__ == '__main__':
  domain_fn, background_knowledge_fn, state_frame_fn, output_scenario_fn = os.sys.argv[1:]
  build_scenario(domain_fn, background_knowledge_fn, state_frame_fn, output_scenario_fn)


