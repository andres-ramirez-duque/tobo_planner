import os
from fondparser import parser, predicate, formula
from fondparser.predicate import Predicate
import oyaml as yaml

LOG=False

class StateValueFrame(object):
  def __init__(self, name, values):
    self.name = name
    self.values = values
  def __str__(self):
    return self.name + "={" + ",".join(self.values())+"}"
class MultiValueFrame(StateValueFrame):
  def __init__(self,name,values):
    super(MultiValueFrame, self).__init__(name,values)
class BooleanValueFrame(StateValueFrame):
  def __init__(self,name):
    super(BooleanValueFrame, self).__init__(name,(False,True))

class StateManagerProxy(object):
  def get_bool_var(self):
    pass
  def get_multi_var(self):
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
    self.bool_vars = rospy.get_param("/boolean_vars")
    self.multi_vars = rospy.get_param("/multi_vars")
  def get_bool_var_value(self, v):
    if v in self.bool_vars:
      return self.bool_vars[v]
    print ("WARNING: missing key in state manager dictionary: " + str(v))
  def get_multi_var_value(self, v):
    if v in self.multi_vars:
      return self.multi_vars[v]
    print ("WARNING: missing key in state manager dictionary: " + str(v))

def parse_background_knowledge(domain_fn, background_knowledge_fn):
  return parser.Problem(domain_fn, background_knowledge_fn)
  
def parse_state_frame(sffn):
  with open(sffn, "r") as stream:
    frame_dict=yaml.safe_load(stream)

  frames=[]  
  for (bool_k, bool_v) in frame_dict["boolean_vars"].items():
    frames.append(BooleanValueFrame(bool_k))
  mv_values_map = frame_dict["multi_vars_values"]
  for (multi_k, multi_v) in frame_dict["multi_vars"].items():
    frames.append(MultiValueFrame(multi_k, mv_values_map[multi_k]))
  return frames

def parse_state_frame_old(sffn):
  lines = filter(lambda x: not x == "", map(lambda x: x.strip(), open(sffn).readlines()))
  frames=[]
  for line in lines:
    content = line.split(":")
    ftype=content[0]
    if ftype.lower()=="boolean":
      frame = BooleanValueFrame(content[1])
    elif ftype.lower()=="multi":
      name,values=content[1:]
      parsed_values = values.split(",")
      frame = MultiValueFrame(name, parsed_values)
    else:
      print ("Warning, unknown frame type: " + ftype + " in state_builder.py - breaking!")
    frames.append(frame)
  return frames

def make_proposition(P, value):
  content = value.split(" ")
  args = map(lambda o: tuple([o]+ list(P.obj_to_type[o])), content[1:])
  p = predicate.Predicate(content[0], args)
  return formula.Primitive(p)

def add_boolean_value_to_state(P, name, value):
  #assert value is bool
  if str(value).lower()== "true" :
    P.init.args.append(make_proposition(P, name))
def add_multi_value_to_state(P, value):
  P.init.args.append(make_proposition(P, value))

#def complete_state_description_no_frame(P):
#  bool_vars = rospy.get_param("/boolean_vars")
#  multi_vars = rospy.get_param("/multi_vars")
#  for k in bool_vars:
#    add_boolean_value_to_state(P, k, bool_vars[k])
#  for k in multi_var:
#    add_multi_value_to_state(P, multi_vars[k])

def complete_state_description(P, state_frames, state_manager):
  for frame in state_frames :
    if isinstance (frame, BooleanValueFrame):
      add_boolean_value_to_state(P, frame.name, state_manager.get_bool_var_value(frame.name))
    elif isinstance (frame, MultiValueFrame):
      add_multi_value_to_state(P, state_manager.get_multi_var_value(frame.name))
    else :
      print ("WARNING: unknown variable type ", frame.__class__)

def get_state_manager(is_ros, frames):
  sm=None
  if is_ros:
    sm=RosStateManagerProxy()
  else:
    sm=DummyStateManagerProxy(frames)
  return sm

def make_current_scenario(domain_fn, background_knowledge_fn, state_frame_fn, is_ros):
  P = parse_background_knowledge(domain_fn, background_knowledge_fn)
  state_frames = parse_state_frame(state_frame_fn)
  state_manager = get_state_manager(is_ros, state_frames)
  complete_state_description(P, state_frames, state_manager)
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
  P = make_current_scenario(domain_fn, background_knowledge_fn, state_frame_fn, is_ros)
  output_scenario(P, output_scenario_fn, is_costed)

if __name__ == '__main__':
  domain_fn, background_knowledge_fn, state_frame_fn, output_scenario_fn = os.sys.argv[1:]
  build_scenario(domain_fn, background_knowledge_fn, state_frame_fn, output_scenario_fn)


