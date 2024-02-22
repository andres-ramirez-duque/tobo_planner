import sys
import int_manager as IM

ADD_DUMMY_STOP_ON_ANY_KEY=True

if __name__ == '__main__':
  #plan = ("doactivity2bold_intro_intronau_introstep_medium_low","progressprocstep1_introstep_preprocedure","goal")
  plan = """readytostart
ivintroduction_intro
introductionpause
ivstartpreprocedure
pameducateonprocedure_ivdescription
ppstartanxietymanagement
ppamtestanxiety
ppamengagementtest
ppmakedivertionplan
ppdivertor_song1
ivcompletepreprocedure
ivquerysitecheck
sceducateonprocedure_strategyenforce
relaxduringcheck_leadmeditation
completesitecheck
readyforprocedure
ivstartprocedure
pimplementdivertionplandivert_dance1
firstcompleteprocedure
procedurepause
ivdebrief_song1
ivfinish_bye
goal""".split()

  
  if len(sys.argv) > 1 and sys.argv[1] == "12":
    pfn, yfn = "params/params0.12.txt", "model0.12/state_frames_scenario.yaml"
  else:
    pfn, yfn = "params/params0.9.txt", "model0.9/state_frames_scenario.yaml"
  im = IM.int_manager(IM.dummy_ros_proxy(plan, pfn, yfn), yfn)
  if ADD_DUMMY_STOP_ON_ANY_KEY:
    from pynput import keyboard
    listener = keyboard.Listener(on_press=im.key_press_handler)
    listener.start()  # start to listen on a separate thread
  im.init()
  
## expected M12 issues:
# hierarchy - no entries
# missing state updates 

