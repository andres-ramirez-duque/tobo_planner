
import os, importlib
import os.path
import cmvalidator as PIVAL
import subprocess
import metric_stripper

LOG=True
DEMO_ROOT=""

USAGE_STRING = """XXX placeholder!
Usage: python planner.py <domain> <problem>

    <domain> and <problem> are the original FOND domain and problem files.

    Example usage: python planner.py domain.pddl p3.pddl

    Caveats:
      * Equality predicates are ignored in the grounding / simulation.
        """
def get_next_action(domain_fn, prob_fn, cmplan_abs_path, pi_fn="human_policy.out", is_costed=False):
  
  if os.path.exists(pi_fn):
    ok,first_action = run_simulator(domain_fn, prob_fn, pi_fn, "prp", is_costed)
  else:
    ok = False
  if ok:
    return first_action
  if LOG:
    print "Policy insufficient for new state, creating new plan!"
  npi_fn = run_planner(domain_fn, prob_fn, cmplan_abs_path)
  ok,first_action = run_simulator(domain_fn, prob_fn, pi_fn, "prp", is_costed)
  if not ok:
    print "WARNING: Incomplete plan generated!"
  return first_action


def run_simulator(dom, prob, sol, interp, is_costed):
  if is_costed :
    ncdom, ncprob = "_no_cost_domain.pddl", "_no_cost_problem.pddl"
    metric_stripper.remove_cost_models(dom, prob, ncdom, ncprob)
    dom, prob = ncdom, ncprob

  return PIVAL.validate(dom, prob, sol, importlib.import_module("validators.%s" % interp))
  
def run_planner(domain_fn, prob, cmplan_abs_path):
  args_plan = [cmplan_abs_path, domain_fn, prob, "--keep-files"]
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

  print get_next_action(dom, prob, solution, "./cmplan")

