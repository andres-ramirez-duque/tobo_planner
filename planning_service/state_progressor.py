
import os, importlib

from fondparser import grounder
from fondparser.formula import *
from normalizer import flatten

import networkx as nx
from networkx.drawing.nx_pydot import write_dot

USAGE_STRING = """
Usage: python validator.py <domain> <problem> <solution> <interpreter>

    <domain> and <problem> are the original FOND domain and problem files.
    <solution> is a file that contains the FOND plan.
    <interpreter> is the module for the <solution> interpreter.
        validators/<interpreter>.py should exist and contain
        (at least) two functions: load and next_action

    Example usage: python validator.py domain.pddl p3.pddl policy.out prp

    Caveats:
      * Equality predicates are ignored in the grounding / simulation.

        """

class State:
    def __init__(self, fluents):
        self.fluents = set(fluents)
        self.hash_val = hash(','.join(sorted(list(map(str, fluents)))))

    def __hash__(self):
        return self.hash_val

    def __eq__(self, other):
        return self.hash_val == other.hash_val

    def is_goal(self, goal):
        return goal <= self.fluents


class VALAction:
    def __init__(self, prec, effs, name, mapping):
        self.name = name
        self.ppres = set(filter(lambda x: x > 0, prec))
        self.npres = set([-1 * i for i in filter(lambda x: x < 0, prec)])
        self.effs = effs
        self.mapping = mapping

    def __str__(self):
        return "%s: +Pre = %s / -Pre = %s / Effs = %s" % (self.name, \
                        str([self.mapping[fl] for fl in self.ppres]), \
                        str([self.mapping[fl] for fl in self.npres]), \
                        str(["(%s --> %s)" % (','.join([self.mapping[fl] for fl in c]), ','.join([self.mapping[fl] for fl in e])) for (c,e) in self.effs]))


def progress_state(dfile, pfile, a):

    print "\nParsing the problem..."

    problem = grounder.GroundProblem(dfile, pfile)

    if a=='':
      return [map(lambda p: p.predicate, problem.init.args),]

    fluents = {}
    unfluents = {}
    runfluents = {}
    index = 1
    for f in problem.fluents:
        fluents[str(f).lower()] = index
        fluents["not(%s)" % str(f).lower()] = -1 * index
        unfluents[index] = str(f).lower()
        unfluents[-1 * index] = "not(%s)" % str(f).lower()
        runfluents[index] = f
        #runfluents[-1 * index] = Not([f]) # don't need negatives for the state..
        index += 1
    actions = {}
    for op in problem.operators:
        if '_' == op.name[-1]:
            op_name = op.name[:-1].lower()
        else:
            op_name = op.name.lower()

        actions[op_name] = [VALAction(_convert_conjunction(fluents, op.precondition),
                                      _convert_cond_effect(fluents, eff), op_name, unfluents)
                            for eff in flatten(op)]

        #print "\n%s\n%s" % (op.name, '\n'.join(map(str, actions[op.name])))

    init_state = State(_convert_conjunction(fluents, problem.init))

    
    print "\nApplying action..."

    out_states=[]
 
    
    for outcome in actions[a]:
        v=progress(init_state, outcome, unfluents)
        out_states.append(sorted([runfluents[i] for i in v.fluents]))
        
    return (sorted(problem.init.args),out_states)


def _convert_cond_effect(mapping, eff):
    if isinstance(eff, And):
        return [_create_cond(mapping, f) for f in filter(lambda x: '=' not in str(x), eff.args)]
    elif isinstance(eff, Primitive) or (isinstance(eff, Not) and isinstance(eff.args[0], Primitive)):
        return [_create_cond(mapping, eff)]
    elif isinstance(eff, When):
        return [_create_cond(mapping, eff)]
    else:
        assert False, "Error: Tried converting a non-standard effect: %s" % str(eff)

def _create_cond(mapping, eff):
    if isinstance(eff, Primitive) or (isinstance(eff, Not) and isinstance(eff.args[0], Primitive)):
        return (set(), set([mapping[str(eff).lower()]]))
    elif isinstance(eff, When):
        return (set(_convert_conjunction(mapping, eff.condition)), set(_convert_conjunction(mapping, eff.result)))
    else:
        assert False, "Error: Tried converting a non-standard single effect: %s" % str(eff)

def _convert_conjunction(mapping, conj):
    if isinstance(conj, And):
        return [mapping[str(f).lower()] for f in filter(lambda x: '=' not in str(x), conj.args)]
    elif isinstance(conj, Primitive) or (isinstance(conj, Not) and isinstance(conj.args[0], Primitive)):
        return [mapping[str(conj).lower()]]
    else:
        assert False, "Error: Tried converting a non-standard conjunction: %s" % str(conj)

def _state_string(mapping, state):
    return '\n'.join(sorted([mapping[i] for i in state.fluents]))

def progress(s, o, m):
    assert o.ppres <= s.fluents and 0 == len(o.npres & s.fluents), \
        "Failed to progress %s:\nPrecondition: %s\nState:\n%s" % \
        (o.name, str(o.ppres), _state_string(m, s))

    #print "\nProgressing the following operator:"
    #print (o)

    adds = set()
    dels = set()
    for eff in o.effs:
        negeff = set(filter(lambda x: x < 0, eff[0]))
        poseff = eff[0] - negeff
        negeff = set(map(lambda x: x * -1, negeff))
        if (poseff <= s.fluents) and 0 == len(negeff & s.fluents):
            for reff in eff[1]:
                if reff < 0:
                    dels.add(reff * -1)
                else:
                    adds.add(reff)

    if 0 != len(adds & dels):
        print "Warning: Conflicting adds and deletes on action %s" % str(o)

    return State(((s.fluents - dels) | adds))


if __name__ == '__main__':
    try:
        (dom, prob, sol, interp) = os.sys.argv[1:]
    except:
        print "\nError with input."
        print USAGE_STRING
        os.sys.exit(1)

    validate(dom, prob, sol, importlib.import_module("validators.%s" % interp))
