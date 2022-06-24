import sys
import int_manager as IM
import espec_parser

if __name__ == '__main__':
  plan0_5 = ("doactivity1bold_intro_intronau_introstep_medium_low","progressprocstep1_introstep_preprocedure","anxietytest_preprocedure", "mitigationactivity_taichi_cognitivebehaviour_preprocedure","progressprocstep2_preprocedure_procedure", "goal")
  plan0_7 = ("doactivity1bold_intro_intronau_introstep_medium_low","progressprocstep1_introstep_preprocedure","anxietytest_preprocedure", "mitigation_preprocedure","doactivitym1aold_taichi_cognitivebehaviour_preprocedure_medium", "progressprocstep2_preprocedure_procedure", "goal")
  
  espec_fn = sys.argv[1]
  im = IM.int_manager(IM.dummy_ros_proxy(plan0_7), espec_fn)
  im.init()
