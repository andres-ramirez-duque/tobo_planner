import os
#from shared_planning import original_model_loader as PDDL_loader
#import shared_planning.pddl_io as PLAN_IO
#from shared_types.planning_types import FuncIncrease, ConjEffect
import planning_helpers
import planning_helpers.original_model_loader as PDDL_loader
import planning_helpers.pddl_io as PLAN_IO
from planning_helpers.planning_types import FuncIncrease, ConjEffect

###############################################################
## Domain
###############################################################


# should be implemented for more effects!
def is_cost_effect(E):
  if isinstance(E, FuncIncrease):
    return E.lhs.func.name == "total-cost"
  return False
      
def remove_action_cost(A):
  conj = []
  if isinstance(A.effect, ConjEffect):
    for E in A.effect.conj:
      if not is_cost_effect(E):
        conj.append(E)
  else:
    if not is_cost_effect(E):
      conj.append(E)
  A.effect = ConjEffect(conj)

# heavy assumptions...
def remove_total_cost_function(D):
  D.functions=None

def remove_cost_from_domain(D):
  for A in D.actions:
    remove_action_cost(A)
  remove_total_cost_function(D)
  D.has_action_costs = False
  if ":action-costs" in D.requirements:
    D.requirements.remove(":action-costs")


###############################################################
## Problem
###############################################################

def remove_cost_from_initial_state(S):
  if not S.funcs == None:
    cost_f=None
    for p in S.funcs:
      if p.name == "total-cost":
        cost_f=p
    if not cost_f==None:
      S.funcs.pop(cost_f)

def remove_cost_from_problem(P):
  remove_cost_from_initial_state(P.initial_state)
  P.has_action_costs = False


###############################################################
## Util
###############################################################

def get_planning_models(domain_fn,  problem_fn): 
  return PDDL_loader.get_planning_model(domain_fn,  problem_fn)

def write_problem(fn, P):
  PLAN_IO.write_out_problem(fn, P, P.initial_state, P.goal, False, False)

def write_domain(fn, D):
  PLAN_IO.write_out_domain(fn, D)

def remove_cost_models(oDfn,oPfn,Dfn,Pfn):
  D,P = get_planning_models(oDfn,oPfn)
  remove_cost_from_domain(D)
  remove_cost_from_problem(P)
  write_domain(Dfn, D)
  write_problem(Pfn, P)

###############################################################
## Main
###############################################################

if __name__ == "__main__":
  oDfn,oPfn,Dfn,Pfn=os.sys.argv[1:]
  remove_cost_models(oDfn,oPfn,Dfn,Pfn)

