import sys
import int_manager as IM
import espec_parser

if __name__ == '__main__':
  plan = ("doactivity1bold_intro_intronau_introstep_medium_low","progressprocstep1_introstep_preprocedure","anxietytest_preprocedure", "doactivity1aold_dance1_distraction_preprocedure_high","progressprocstep2_preprocedure_procedure", "goal")
  espec_fn = sys.argv[1]
  im = IM.int_manager(IM.dummy_ros_proxy(plan), espec_fn)
  im.init()
