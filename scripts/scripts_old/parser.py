
import xml.etree.ElementTree as ET


###################################################################################################################
### Policy XML Parsing
###################################################################################################################

class Node(object):
  def __init__(self, node_id, state_id, transition_id, _xml_node):
    self.node_id = node_id
    self.state_id = state_id
    self.transition_id = transition_id
    self._xml_node = _xml_node

  def __str__(self):
    pass

class ActionNode(Node):
  def __init__(self, node_id, state_id, transition_id, next_node, _xml_node):
    super(ActionNode, self).__init__(node_id, state_id, transition_id, _xml_node)
    self.next_node = next_node
    
  def __str__(self):
    return str(self.node_id) + " [" + str(self.transition_id)+ "]=> " + str(self.next_node)

class SensorNode(Node):
  def __init__(self, node_id, state_id, transition_id, neighbours, _xml_node):
    super(SensorNode, self).__init__(node_id, state_id, transition_id, _xml_node)
    self.neighbours = neighbours
    
  def __str__(self):
    return str(self.node_id) + " [" + str(self.transition_id)+ "]=> " + str(self.neighbours)
    
def parse_node(node_el, is_action):
  node_id=int(node_el.find('node_id').text)
  state_id=int(node_el.find('state_id').text)
  transition_id=int(node_el.find('transition_id').text)
  
  if is_action:
    next_node = int(node_el.find('next_node_id').text)
    return ActionNode(node_id, state_id, transition_id, next_node, node_el)
  else:
    neighbours = {}
    map_el = node_el.find("next_node_map")
    for neigh in map_el.findall('map_entry'):
      nsval = neigh.find('sensor_value').text.upper()
      assert nsval in ("TRUE","FALSE") # assume boolean for now
      nsval = nsval=="TRUE"
      nnid = int(neigh.find('next_node_id').text)
      neighbours[nsval]=nnid
    return SensorNode(node_id, state_id, transition_id, neighbours, node_el)

def parse_policy_xml(p_fn):
  policy_tree = ET.parse(p_fn)
  policy_root = policy_tree.getroot()
  init_node = int(policy_root.find('init_node_id').text)
  node_d = {}
  for node_el in policy_root.findall('action_transition') :
    node = parse_node(node_el, is_action=True)
    node_d[node.node_id] = node
  for node_el in policy_root.findall('sensor_transition') :
    node = parse_node(node_el, is_action=False)
    node_d[node.node_id] = node
  return node_d, init_node



###################################################################################################################
### Transition XML Parsing
###################################################################################################################


class Transition(object):
  def __init__(self, transition_id, operator, parameters, _xml_node):
    self.transition_id = transition_id
    self.operator = operator
    self.parameters = parameters
    self._xml_node = _xml_node

  def __str__(self):
    return "[" + str(self.transition_id) + "] " + self.operator + " " + " ".join(self.parameters)

class Action(Transition):
  def __init__(self, transition_id, operator, parameters, _xml_node):
    super(Action, self).__init__(transition_id, operator, parameters, _xml_node)

class Sensor(Transition):
  def __init__(self, transition_id, operator, parameters, _xml_node):
    super(Sensor, self).__init__(transition_id, operator, parameters, _xml_node)

def parse_transition(t_el, is_action):
  transition_id=int(t_el.find('id').text)
  operator=t_el.find('operator').text
  parameters = []
  parameters_el=t_el.find('parameters')
  for i, p_el in enumerate(parameters_el.findall('parameter')):
    assert int(p_el.attrib["index"]) == i
    parameters.append(p_el.text)
  if is_action:
    transition=Action(transition_id, operator, parameters, t_el)
  else:
    transition=Sensor(transition_id, operator, parameters, t_el)
  return transition

def parse_transitions_xml(t_fn):
  transition_tree = ET.parse(t_fn)
  transition_root = transition_tree.getroot()

  transition_map = {}
  for t_el in transition_root.findall('action') :
    t=parse_transition(t_el, is_action=True)
    transition_map[t.transition_id]=t
  for t_el in transition_root.findall('sensor') :
    t=parse_transition(t_el, is_action=False)
    transition_map[t.transition_id]=t
  return transition_map
  
if __name__ == "__main__":
  import sys
  #p_fn = sys.argv[1]
  #node_d,init = parse_policy_xml(p_fn)
  #print node_d[init]
  t_fn = sys.argv[1]
  transition_map=parse_transitions_xml(t_fn)
  print transition_map[1]
  

