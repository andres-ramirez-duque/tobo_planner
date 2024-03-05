

script = {"finish": [["goal",[]]],
          "debrief": [["ivfinish",["bye"]]],
          "procedure": [["waitforproceduretoend",[]],["ivdebrief",["happy"]]],
          "sitecheck": [["readyforprocedure",[]], ["startprocedure",[]],["pimplementdivertionplandivert",["bruno"]]],
          "preprocedure": [["ivquerysitecheck",[]],["ivstartsitecheck",[]],["relaxduringcheck",["taichi"]]],
          "intro": [["introductionpause",[]], ["ivstartpreprocedure",[]],["ppdivertor",["look"]]],
          "pre": [["readytostart",[]],["ivintroduction",["intro"]]]
          }
l = ["pre", "intro", "preprocedure", "sitecheck", "procedure", "debrief", "finish"]

def identify_stage(prefix):
  ops = map(lambda (op,params): op, prefix)
  stage = "pre"
  if "ivintroduction" in ops :
    stage = "intro"
  else: return stage
  if "ivstartpreprocedure" in ops:
    stage = "preprocedure"
  else: return stage
  if "ivstartsitecheck" in ops:
    stage = "sitecheck"
  else: return stage
  if "ivstartprocedure" in ops:
    stage = "procedure"
  else: return stage
  if "ivdebrief" in ops:
    stage = "debrief"
  else: return stage
  if "ivfinish" in ops:
    stage = "finish"
  return stage

def get_suffix(stage):
  return [a for ls in map(lambda x: script[x], l[l.index(stage):]) for a in ls]

def sequence_continuer_generator(prefix):
  stage = identify_stage(prefix)
  return get_suffix(stage)



