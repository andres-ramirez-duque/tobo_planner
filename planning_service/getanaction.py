
import os, sys, subprocess

############################################################################################################
### defaults 
############################################################################################################

deadend_detection=False
keep_files=False
DEMO_ROOT=""
PRP_LOCATIONS=["/home/al/planners/planner-for-relevant-policies"]
for loc in PRP_LOCATIONS:
  if os.path.exists(loc):
    PRP_ROOT=loc
    break
else:
  print "ERROR: Cannot locate PRP in getanaction.py - add to PRP_LOCATIONS"
  sys.exit(1)

scenario_fn="pout.pddl"
solution_fn="aplan.out"

USAGE_STRING = """
Usage: python getanaction.py <domain> <background knowledge> <state frames>

    <domain> and <background knowledge> are a FOND domain and partial problem files. In particular, <background knowledge> provides the static part of the scenario.
    <state frames> is a set of variables that will be requested from the state manager to complete the state. This is the dynamic part of the state.

    Example usage: python getanaction.py model0.2/domain_plan.pddl model0.2/scenario_background_knowledge.pddl model0.2/state_frames_scenario.txt

    Caveats:
      * In PRP, equality predicates are ignored in the grounding / simulation.
        """



############################################################################################################
### local imports and ugly path stuff
############################################################################################################

# Add PRP to the path
sys.path.append(PRP_ROOT+"/prp-scripts")
import state_builder, make_prp_runner, planner



############################################################################################################
### get an action functions
############################################################################################################

def build_scenario(domain_fn, background_knowledge_fn, state_frame_fn, scenario_fn):
  state_builder.build_scenario(domain_fn, background_knowledge_fn, state_frame_fn, scenario_fn)

def get_next_action(domain_fn, scenario_fn, solution_fn, prp_root):
  make_prp_runner.make_prp_runner(prp_root, solution_fn, deadend_detection)
  run_a_command( ["chmod", "u+x", "cmplan"])
  return planner.get_next_action(domain_fn, scenario_fn, solution_fn)

def run_a_command(command_args):
  print " ".join(map(lambda x: str(x), command_args))
  exit_c = subprocess.call(command_args)
  return exit_c

def remove_if_exists(fn):
  if os.path.exists(fn):
    os.remove(fn)

def cleanup(prp_root):
  if not keep_files:
    run_a_command([PRP_ROOT+"/src/cleanup"])
    remove_if_exists("policy.out")
    remove_if_exists("human_policy.out")
    remove_if_exists("graph.dot")
    remove_if_exists("action.map")
    remove_if_exists("graph.png")
    remove_if_exists("unhandled.states")
    remove_if_exists("policy.fsap")



############################################################################################################
### main
############################################################################################################

if __name__ == '__main__':
  DEMO_ROOT=os.path.dirname(os.path.abspath(os.sys.argv[0]))
  try:
    (domain_fn, background_knowledge_fn, state_frame_fn) = os.sys.argv[1:]
  except:
    print "\nError with input."
    print USAGE_STRING
    os.sys.exit(1)
  
  build_scenario(domain_fn, background_knowledge_fn, state_frame_fn, scenario_fn)
  print "THE ACTION: " + str(get_next_action(domain_fn, scenario_fn, solution_fn, PRP_ROOT))
  cleanup(PRP_ROOT)
  
  
  
