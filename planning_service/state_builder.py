import os
import dummy_state_manager as state_manager
from fondparser import parser, predicate, formula

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


def parse_background_knowledge(domain_fn, background_knowledge_fn):
  return parser.Problem(domain_fn, background_knowledge_fn)
  
def parse_state_frame(sffn):
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
  if str(value).lower()=="true" :
    P.init.args.append(make_proposition(P, name))
def add_multi_value_to_state(P, value):
  P.init.args.append(make_proposition(P, value))

def complete_state_description(P, state_frames):
  for frame in state_frames :
    value = state_manager.request_feature_value(frame.name, frame.values)
    if LOG:
      print ("Current Value: " + frame.name + "=" + str(value))
    if isinstance (frame, BooleanValueFrame):
      add_boolean_value_to_state(P, frame.name, value)
    elif isinstance (frame, MultiValueFrame):
      add_multi_value_to_state(P, value)
    else:
      print "Shouldn't be here! state_builder.py"
      assert False

def make_current_scenario(domain_fn, background_knowledge_fn, state_frame_fn):
  P = parse_background_knowledge(domain_fn, background_knowledge_fn)
  state_frames = parse_state_frame(state_frame_fn)
  complete_state_description(P, state_frames)
  return P

def output_scenario(P, output_scenario_fn):
  P._export_problem(open(output_scenario_fn,'w'))

def build_scenario(domain_fn, background_knowledge_fn, state_frame_fn, output_scenario_fn):
  p = make_current_scenario(domain_fn, background_knowledge_fn, state_frame_fn)
  output_scenario(p, output_scenario_fn)

if __name__ == '__main__':
  domain_fn, background_knowledge_fn, state_frame_fn, output_scenario_fn = os.sys.argv[1:]
  build_scenario(domain_fn, background_knowledge_fn, state_frame_fn, output_scenario_fn)


