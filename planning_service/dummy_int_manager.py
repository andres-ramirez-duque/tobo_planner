import int_manager as IM

if __name__ == '__main__':
  plan = ("doactivity2bold_intro_intronau_introstep_medium_low","progressprocstep1_introstep_preprocedure","goal")
  im = IM.int_manager(IM.dummy_ros_proxy(plan))
  im.init()
