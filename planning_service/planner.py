
import os, importlib
import os.path
import cmvalidator as PIVAL
import subprocess

LOG=True
DEMO_ROOT=""

USAGE_STRING = """XXX placeholder!
Usage: python planner.py <domain> <problem>

    <domain> and <problem> are the original FOND domain and problem files.

    Example usage: python planner.py domain.pddl p3.pddl

    Caveats:
      * Equality predicates are ignored in the grounding / simulation.
        """
def get_next_action(domain_fn, prob_fn, pi_fn="human_policy.out"):
  
  if os.path.exists(pi_fn):
    ok,first_action = run_simulator(domain_fn, prob_fn, pi_fn, "prp")
  else:
    ok = False
  if ok:
    return first_action
  if LOG:
    print "Policy insufficient for new state, creating new plan!"
  npi_fn = run_planner(domain_fn, prob_fn)
  ok,first_action = run_simulator(domain_fn, prob_fn, pi_fn, "prp")
  if not ok:
    print "WARNING: Incomplete plan generated!"
  return first_action


def run_simulator(dom, prob, sol, interp):
  return PIVAL.validate(dom, prob, sol, importlib.import_module("validators.%s" % interp))
  
def run_planner(domain_fn, prob):
  plan_arg = DEMO_ROOT + "/cmplan"
  args_plan = [plan_arg, domain_fn, prob, "--keep-files"]
  if LOG :
    print " ".join(map(lambda x: str(x), args_plan))
  exit_c = subprocess.call(args_plan)

if __name__ == '__main__':
  DEMO_ROOT=os.path.dirname(os.path.abspath(os.sys.argv[0]))
  try:
    (dom, prob, solution) = os.sys.argv[1:]
  except:
    print "\nError with input."
    print USAGE_STRING
    os.sys.exit(1)

  print get_next_action(dom, prob, solution)

