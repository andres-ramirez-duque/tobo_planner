import logging
import threading
import time

## TODO
# keys for timers
# infallible thread protection
# anxiety dummy idea
# how do we consolidate if there are more than one thing executing? Well, we could have a execution timer? Then we need to record aspects and defaults for all remaining. 

class Enum(object): 
  def __init__(self, tupleList):
    self.tupleList = tupleList
    
  def __getattr__(self, name):
    return self.tupleList.index(name)
  
  def __getitem__(self, key):
    return self.tupleList[key]
    
manager_status_enum = Enum(("planning", "executing", "idle"))
manager_status = manager_status_enum.idle
man_status_lock = threading.Lock()
planner_status_enum = Enum(("active", "idle"))
planner_status = planner_status_enum.idle
nau_status_enum = Enum(("active", "idle"))
nau_status = nau_status_enum.idle
web_server_status_enum = Enum(("active", "idle"))
web_server_status = web_server_status_enum.idle
interaction_status_enum = Enum(("before", "during", "after"))
interaction_status = interaction_status_enum.before
procedure_status_enum = Enum(("null", "introstep", "preprocedure", "procedure", "debrief", "end"))
procedure_status = procedure_status_enum.null
interaction_step_status_enum = Enum(("anxiety_test", "nau", "transition"))
interaction_step_status = None


class Timer(object):

  def __init__(self, t):
    self.t=t
  
  def start(self, args=(1,)):
    x = threading.Thread(target=self.thread_function, args=args)
    x.start()
    
  def thread_function(self, args):
    print str(args), "---> going to sleep..."
    time.sleep(self.t)
    self._trigger(args)
    
  def _trigger(self, args):
    pass

class MessageGiver(Timer):
  def __init__(self, t, s, m):
    super(MessageGiver, self).__init__(t)
    self.s=s
    self.m=m
  
  def _trigger(self, args):
    apply(self.s, (self.m, args))
    #on_timer_event(self.m, args) # source?

class WebServer(MessageGiver):
  def __init__(self, t, s):
    super(WebServer, self).__init__(t, s, None)
  
  def ask_for_user_input(self, m, args = (1,)):
    self.m = m
    self.start(args)
  
  def disable_user_input(self, m):
    pass

class Planner(Timer):
  def __init__(self, t, s, steps):
    super(Planner, self).__init__(t)
    self.s=s
    self.steps = steps
    self.next_id = 0
    
  def _trigger(self, args):
    #self.s.on_timer_event(self.m)
    apply(self.s, (self.steps[self.next_id],args))
    self.next_id+=1

class int_manager(object):
  def __init__(self, plan):
    self.web_server = WebServer(5, self.on_timer_event)
    self.planner = Planner(2, self.get_message, plan)
    self.message_keys = []

  def ask_user_progress_proc_step(self, message, k):
    self.web_server.ask_for_user_input(message, (k,))

  def process_progress_proc_step_action(self, message):
    global web_server_status
    web_server_status = web_server_status_enum.active
    bits = message.split("_")
    s1,s2 = bits[1:3]
    k = "progresser"
    web_server = MessageGiver(10, self.on_timer_event, s1).start((k,))
    self.ask_user_progress_proc_step(s2, k)


  def get_message(self, message, args): # a broadcast from the planner
    global planner_status
    print "++++ Received action: " + message
    planner_status = planner_status_enum.idle
    if message.startswith("action:progressprocstep") :
      self.process_progress_proc_step_action(message)
    if message.startswith("action:goal"):
      global interaction_status
      interaction_status = interaction_status_enum.after

  def start_action_chain(self):
    global planner_status
    print "+ Calling planner.."
    planner_status = planner_status_enum.active
    self.planner.start()

  def on_timer_event(self, message, k):
    if not k in self.message_keys:
      global web_server_status
      web_server_status = web_server_status_enum.idle
      #web_server.disable_user_input(message)
      self.message_keys.append(k)
      print "++++ On timer event: " + str(message) + " from: ", k
    else:
      print "++++ Ignoring: ", k

  def init(self):
    global interaction_status
    if interaction_status == interaction_status_enum.before:
      interaction_status = interaction_status_enum.during
  
    while not interaction_status == interaction_status_enum.after:
      print (planner_status, nau_status, web_server_status)
      if planner_status == planner_status_enum.idle and nau_status == nau_status_enum.idle and web_server_status == web_server_status_enum.idle:
      
        if interaction_status == interaction_status_enum.during:
          self.start_action_chain()
      time.sleep(5)
    print "=== Completed Int manager loop ====="
  
  
def test():
  get_message("action:progressprocstep1_introstep_preprocedure")


#test()

im = int_manager(("action:doactivity2bold_intro_intronau_introstep_medium_low","action:progressprocstep1_introstep_preprocedure","action:goal"))
im.init()



