import sys
import int_manager as IM
import espec_parser

if __name__ == '__main__':
  plan = ("doactivity2bold_intro_intronau_introstep_medium_low","progressprocstep1_introstep_preprocedure","goal")
  espec_fn = sys.argv[1]
  im = IM.int_manager(IM.dummy_ros_proxy(plan), espec_fn)
  im.init()
