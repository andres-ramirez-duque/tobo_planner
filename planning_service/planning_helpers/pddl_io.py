
import parder, FileUtil, planning_types
from parder import PddlDomainParser, PddlProblemParser, PDDLPlanParser
from planning_types import ConjGoal, PropGoal, InstDiscreteAction, NegPropGoal
LOG=False

domain_parser = PddlDomainParser()
problem_parser = PddlProblemParser()
plan_parser = PDDLPlanParser()


def read_domain(domain_name) :
  if LOG:
    print "Reading in domain: " + domain_name
  domain_lines = map(lambda x: x.decode("utf-8-sig").encode("utf-8").replace("\t","  "), FileUtil.readlines(domain_name, ";"))
  return domain_parser.parse(domain_lines)

def read_problem(problem_name) :
  problem_lines = map(lambda x: x.decode("utf-8-sig").encode("utf-8").replace("\t","  "), FileUtil.readlines(problem_name,";"))
  return problem_parser.parse(problem_lines)

def read_plan(plan_name) :
  plan_lines = map(lambda x: x.decode("utf-8-sig").encode("utf-8").replace("\t","  "), FileUtil.readlines(plan_name, ";"))
  return plan_parser.parse(plan_lines, domain)


# think I'd need a problem too
def write_out_problem(problem_path, supporting_problem, s, g, METRIC=True, HIDDEN=False) :
  TAB = "  "
  headerStr = "(define (problem " + supporting_problem.name + ")\n"
  headerStr += TAB + "(:domain " + supporting_problem.domain + ")\n"
  headerStr += TAB + "(:objects \n"
  # objects
  for otype in set(map(lambda (o,t): t, supporting_problem.objects.items())):
    headerStr+= 2*TAB + " ".join(map(lambda (o,t): o, filter(lambda (o,t): t==otype, supporting_problem.objects.items()))) + " - " + otype + "\n"
  #headerStr += "".join(map(lambda (o,t): TAB+TAB + o + " - " + t + "\n", baseline_problem.objects.items()))
  headerStr += TAB + ")\n"
  headerStr += TAB + "(:init\n"
  # stationary state
  headerStr += "".join(map(lambda x: 2*TAB + str(x.pddl_str()) + "\n", s.props))
  headerStr += "".join(map(lambda (p,v): 2*TAB +"(= " + str(p.pddl_str()) + " " + str(v) + ")\n",  s.funcs.items()))
  if METRIC :
    if len(filter(lambda p: p.name=="total-cost", s.funcs.keys()))==0:
      headerStr += 2*TAB + "(= (total-cost) 0)\n"
  headerStr += TAB + ")\n"
  if HIDDEN:
    headerStr += TAB + "(:hidden )\n"
  goalStr = TAB + "(:goal\n" + TAB + str(g) + "\n" +TAB + ")"
  if METRIC :
    metricStr = "(:metric minimize (total-cost))"
  ### Problem str ###

  modelStr = headerStr
  #modelStr += TAB+")\n" #??
  modelStr += goalStr + "\n"
  if METRIC :
    modelStr += TAB + metricStr + "\n"
  FileUtil.write(problem_path, modelStr+")")


def write_out_domain(domain_path, domain) :
  domain_str = str(domain)
  FileUtil.write(domain_path, domain_str)


def write_out_plan(plan_path, plan) :
  plan_str = "\n".join(map(lambda x: " ".join(x), plan))
  FileUtil.write(plan_path, plan_str)

def parse_plan(lines) :
  return map(lambda l: l.split(" "), filter(lambda x: not "." in x, filter(lambda l: not l == "", map(lambda l: l.strip(), lines))))

def parse_raw_plan(lines, metric=False) :
  return resultExtractor(lines)


def metricResultExtractor(lines) :
  plan = []
  suckUpPlan = success = False
  n = -1; c = -1
  for line in lines :
    if "ff: goal can be simplified to TRUE. The empty plan solves it" in line :
      return [], True
    elif "ff: goal can be simplified to FALSE. No plan will solve it" in line:
      return None, False
    if suckUpPlan :
      if "time spent:" in line or line == '\n' or line == "" or "plan cost:" in line :
        suckUpPlan = False
      else :
        if ":" in line :
          cols = line.split(":")
          plan.append(cols[1].strip().split(" "))
          print "CHEWING:", cols[1].strip().split(" ")
    if "ff: found legal plan as follows" in line :
      print "NOTICED: SUCCESS"
      success = True
      suckUpPlan = True
    if "seconds total time" in line :
      t = float(line[:-20])
  if not success :
    plan = None
  return plan, success

def resultExtractor(lines) :
  plan = []
  suckUpPlan = success = False
  n = -1; c = -1
  for line in lines :
    if "ff: goal can be simplified to TRUE. The empty plan solves it" in line :
      return [], True
    elif "ff: goal can be simplified to FALSE. No plan will solve it" in line:
      return None, False
    if suckUpPlan :
      if "time spent:" in line or line == '\n' or line == "" or "plan cost:" in line :
        suckUpPlan = False
      else :
        if ":" in line :
          cols = line.split(":")
          plan.append(cols[1].strip().split(" "))
    if "ff: found legal plan as follows" in line :
      success = True
      suckUpPlan = True
    if "seconds total time" in line :
      t = float(line[:-20])
  if not success :
    plan = None
  return plan, success


# written out for goal recognition software
def write_out_goal(goal_path, goal) :
  if isinstance(goal, ConjGoal) :
    goal_str = ",".join(map(lambda g: str(g), goal.conj))
  elif isinstance(goal, list) :
    goal_str = ",".join(map(lambda g: str(g), goal))
  else :
    print "Not implemented: pddl_io goal writer currently expects a conjunction.."
    goal_str = ""
  FileUtil.write(goal_path, goal_str)
