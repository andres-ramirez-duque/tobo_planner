#!/usr/bin/env python
import logging
import threading
import time
import rospy

from std_msgs.msg import String
from tobo_planner.msg import action_chain
from tobo_planner.msg import web_chain
from tobo_planner.srv import PlannerMode,PlannerModeResponse
from naoqi_bridge_msgs.srv import SetString



LOG=True
ACTION_CHAIN_LAUNCHER_PAUSE=5

class Enum(object): 
  def __init__(self, tupleList):
    self.tupleList = tupleList
    
  def __getattr__(self, name):
    return self.tupleList.index(name)
  
  def __getitem__(self, key):
    return self.tupleList[key]

def key_maker(obj, t, indx):
  return obj + "." + t + "." + str(indx)
def key_deconstruct(k):
  bits = k.split(".")
  return bits[0], bits[1], int(bits[2])

def parse_action_str(message):
  bits=message.split("_")
  return bits[0][len("action:"):], bits[1:]

"""
The following are options for the internal representation of the statuses of parts of the system.
We need to smooth out some of the threading issues. E.g., time delay between planner becoming idle and nau becoming active.
To do this we assume a repeated sequence of: idle, planning, executing.
At the start of execution a set of flags are raised and an execution timer is started.
On all flags being lowered, the execution is finished and the status is idle.
On the execution timer completing, if all flags are lowered then the signal is ignored,
If flags are still raised then the relevant processes are halted. The flags are checked again and any for any raised the default action is taken.
"""
    
manager_status_enum = Enum(("before","planning","executing","idle","after"))
planner_status_enum = Enum(("active", "idle"))
nau_status_enum = Enum(("active", "idle"))
web_server_status_enum = Enum(("active", "idle"))
procedure_status_enum = Enum(("null", "introstep", "preprocedure", "procedure", "debrief", "end"))
interaction_step_status_enum = Enum(("anxiety_test", "nau", "transition"))





class Timer(object):

  def __init__(self, t):
    self.t=t
  
  def start(self, args=(1,)):
    x = threading.Thread(target=self.thread_function, args=args)
    x.start()
    
  def thread_function(self, args):
    if LOG:
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

class DummyWebServer(MessageGiver):
  def __init__(self, t, s):
    super(DummyWebServer, self).__init__(t, s, None)
    self.web_server_status = web_server_status_enum.idle
  
  def ask_for_user_input(self, options, default, timeout=10, args = (1,)):
    self.m = default
    self.start(args)
  
  def disable_user_input(self, m):
    pass

class WebServer(object):
  def __init__(self, s):
    self.web_chain_sub = rospy.Subscriber("/request",web_chain,s)    
    self.web_chain_pub = rospy.Publisher('/listener_req', web_chain, queue_size=10)
  
  def ask_for_user_input(self, options, default, timeout=10, args = (1,)):
    obj, label, indx = key_deconstruct(args[0])
    msg = web_chain()
    msg.plan_step = indx
    msg.request_type = label
    msg.parameters = options
    msg.duration = timeout #XXX self.op_timeout["progressprocstep"]
    self.web_chain_pub.publish(msg)
    
  
  def disable_user_input(self, m):
    pass

class DummyPlanner(Timer):
  def __init__(self, t, s, steps):
    super(DummyPlanner, self).__init__(t)
    self.s=s
    self.steps = steps
    self.next_id = 0
    self.planner_status = planner_status_enum.idle
    
  def _trigger(self, args):
    apply(self.s, (self.steps[self.next_id],args))
    self.next_id+=1

  def is_active(self) :
    return not self.planner_status == planner_status_enum.idle

class Planner(object):
  def __init__(self, s):
    self.action_sub = rospy.Subscriber("/next_action", action_chain, s)
    self.next_id = 0
    
  def start(self, args):
    print "%%%%%%%%%%%%% PASSING INTO PLANNER", args
    k = key_deconstruct(args[0])
    self.get_an_action_client("chicken", k[2])

  def get_an_action_client(self, planner_mode,plan_step):
    rospy.wait_for_service('get_an_action')
    try:
      get_an_action = rospy.ServiceProxy('get_an_action', PlannerMode)
      resp = get_an_action(planner_mode,plan_step)
      return resp.planner_ok
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)


class service_provider(object):
  def __init__(self):
    self.action_broadcast_f = None
    self.webserver_broadcast_f = None
    self._last_tag = None
  def request_action(self, tag):
    self._last_tag = tag
    self._request_action()
  def ask_for_user_input(self, options, default, timeout, k):
    self._last_tag = k
    self._ask_for_user_input(options, default, timeout, k)
    
  def register_for_action_broadcast(self, callback):
    self.action_broadcast_f = callback
  def register_for_webserver_broadcast(self, callback):
    self.webserver_broadcast_f = callback
  def on_received_planner_action(self, message, t=None):
    apply(self.action_broadcast_f, (message, self._last_tag))
  def on_received_webserver_message(self, message, t=None):
    apply(self.webserver_broadcast_f, (message, self._last_tag))
  def disable_user_info_request(self, tag):
    pass
  def set_parameter(self, path, v):
    print "[SP] Set: " + path + " to: " + str(v)


class dummy_ros_proxy(service_provider):
  def __init__(self, plan):
    super(dummy_ros_proxy, self).__init__()
    self.planner = Planner(2, self.on_received_planner_action, plan)
    self.web_server = WebServer(10, self.on_received_webserver_message)
  def _request_action(self):
    self.planner.start((self._last_tag,))
  def _ask_for_user_input(self, options, default, timeout, k):
    self.web_server.ask_for_user_input(options, default, timeout, (k,))

class ros_proxy(service_provider):
  def __init__(self):
    super(ros_proxy, self).__init__()
    self.planner = Planner(self.on_received_planner_action)
    self.web_server = WebServer(self.on_received_webserver_message)
  def _request_action(self):
    self.planner.start((self._last_tag,))
  def _ask_for_user_input(self, options, default, timeout, k):
    self.web_server.ask_for_user_input(options, default, timeout, (k,))

  


class int_manager(object):
  def __init__(self, service_provider):
    self.service_provider = service_provider
    self.service_provider.register_for_action_broadcast(self.get_message)
    self.service_provider.register_for_webserver_broadcast(self.webserver_message)
    self.active_requests = []
    self.request_defaults = {}
    self.active_requests_lock = threading.Lock()
    
    self.status = manager_status_enum.before
    self.status_lock = threading.Lock()
    self.procedure_status = procedure_status_enum.null # XXX TODO
    self.interaction_step_status = None # XXX TODO
    
    self.counter=0
    self.init_timeouts()
    
  def init_timeouts(self):
    self.op_timeout = {"progressprocstep": 10,
                       "doactivity": 10,
                       "anxietytest": 10,
                       "idle": 10
                       }

  def ask_user_progress_proc_step(self, options, default, timeout, k):
    self.service_provider.ask_for_user_input(options, default, timeout, k)

  def process_progress_proc_step_action(self, op, params,t):
    s1,s2 = params[0:2]
    default = s1
    label = "stage progression"
    self.ask_user_progress_proc_step((s1,s2), s2, t, key_maker("web server",label, self.counter))
    self.add_flag(label, default)

  def process_anxiety_test(self, op, params, t):
    s1 = params[0]
    default = op+"_"+ s1
    label = "anxiety test"
    self.ask_user_progress_proc_step(("true","false"), "false", t, key_maker("web server",label, self.counter))
    self.add_flag(label, default)

  def process_do_activity_action(self, op, params):
    pass

  def process_action_execution(self, op, params):
    self._current_action = self.reconstruct_action_str(op, params)
    self.set_status_if_in_one_of(manager_status_enum.executing, (manager_status_enum.planning,))
    if op.startswith("progressprocstep") :
      t = self.op_timeout["progressprocstep"]
      self.process_progress_proc_step_action(op,params,t)
    elif op.startswith("doactivity"):
      self.process_do_activity_action(op,params)
      t = self.op_timeout["doactivity"]
    elif op.startswith("mitigationactivity"):
      self.process_do_activity_action(op,params)
      t = self.op_timeout["doactivity"]
    elif op.startswith("idle"):
      t = self.op_timeout["idle"]
    elif op.startswith("anxietytest"):
      t = self.op_timeout["anxietytest"]
      self.process_anxiety_test(op,params, t)
    elif op.startswith("goal"):
      if not self.set_status_if_in_one_of(manager_status_enum.after, (manager_status_enum.executing,)):
        print "WARNING: goal achieved, but manager lost.."
      return
    else:
      print "WARNING: unknown action: " + message
      return
    MessageGiver(t, self.on_timer_event, None).start((key_maker("manager","timeout",self.counter),))
    
  def reconstruct_action_str(self, op, params):
    return op+"_"+"_".join(params)
    
  def get_message(self, action_message, dummy): # a broadcast from the planner
    op = action_message.action_type 
    params = action_message.parameters
    indx = action_message.plan_step
    self.process_action_execution(op, params)

  def add_flag(self, flag, default):
    self.active_requests_lock.acquire()
    self.active_requests.append(flag)
    self.request_defaults[flag]=default
    self.active_requests_lock.release()

  def incr_counter_and_clear_flags(self):
    self.active_requests_lock.acquire()
    self.counter+=1
    del self.active_requests[:]
    self.active_requests_lock.release()
    
  def is_active_flag(self, flag):
      self.active_requests_lock.acquire()
      b = flag in self.active_requests
      self.active_requests_lock.release()
      return b
    
  def start_action_chain(self):
    self.incr_counter_and_clear_flags()

    if LOG:
      print "+ Calling planner..", self.counter
    if self.set_status_if_in_one_of(manager_status_enum.planning, (manager_status_enum.idle,)):
      self.service_provider.request_action(key_maker("planner","action request",self.counter))

  def process_request_reply(self, flag, message):
    if flag == "stage progression":
      path_to_stage_param="parameters/multi_vars/proc_stage" # maybe just a yaml parameter?
      self.service_provider.set_parameter(path_to_stage_param, message)
    else:
      print "TODO: do something about ", flag, message

  def record_if_requests_completed(self):
    self.active_requests_lock.acquire()
    self._record_if_requests_completed()
    self.active_requests_lock.release()
  def _record_if_requests_completed(self):
    if len(self.active_requests) == 0:
      path_to_stage_param="/parameters/last_executed_action" # maybe just a yaml parameter?
      rospy.set_param(path_to_stage_param, self._current_action)
      self._current_action=None
      self.set_status_if_in_one_of(manager_status_enum.idle, (manager_status_enum.executing,))
  
  def handle_disengagements(self, flag):
    if flag == "stage progression":
      self.service_provider.disable_user_info_request(flag)
  
  def handle_timeout(self, indx, message):
    if self.status_is_one_of((manager_status_enum.executing,)):
      self.active_requests_lock.acquire()
      if self.counter == indx :
        if LOG:
          print "@TIMEOUT:"
        for flag in self.active_requests:
          #print "TODO: do something about ", flag, self.request_defaults[flag]
          self.process_request_reply(flag, self.request_defaults[flag])
          self.handle_disengagements(flag)
            
        del self.active_requests[:]
        self._record_if_requests_completed()
      else:
        if LOG:
          print "Ignoring timeout - previous step.."
      self.active_requests_lock.release()
    else:
        if LOG:
          print "Ignoring timeout - no longer executing.."

  def webserver_message(self, web_message, dummy):
    #print k
    #obj, t, indx = key_deconstruct(k)
    message = str(web_message.parameters[0])
    t = web_message.request_type
    indx = web_message.plan_step
    
    if self.remove_request_if_active(t, indx):
      if LOG:
        print "++++ On timer event: " + str(message)
      self.process_request_reply(t, message)
      self.record_if_requests_completed()
    else:
      if LOG:
        print "++++ Ignoring: ", k

  def on_timer_event(self, message, k):
    obj, t, indx = key_deconstruct(k)
    if obj == "manager":
      if t == "timeout":
        self.handle_timeout(indx, message)
      else:
        print "WARNING: unknown manager type: ", message, k

  def remove_request_if_active(self, flag, indx):
    # Still a fresh request, if: executing, same index and flag is active
    b=False
    if self.status_is_one_of((manager_status_enum.executing,)):
      self.active_requests_lock.acquire()
      
      if self.counter == indx :
        if flag in self.active_requests:
          b = True
          self.active_requests.remove(flag)
      
      self.active_requests_lock.release()
    return b

  def set_status_if_in_one_of(self, status, statuses):
    self.status_lock.acquire()
    b = self.status in statuses 
    if b:
      self.status = status

    self.status_lock.release()
    return b

  def is_ready(self):
    return self.status_is_one_of((manager_status_enum.idle,))

  def is_finished(self):
    return self.status_is_one_of((manager_status_enum.after,))

  def status_is_one_of(self, statuses):
    self.status_lock.acquire()
    b = self.status in statuses
    self.status_lock.release()
    return b

  def get_status_str(self):
    self.status_lock.acquire()
    s = manager_status_enum[self.status]
    self.status_lock.release()
    return s

  def _start_action_chain_when_appropriate(self):
    while not self.is_finished():
      print "@@@@@@MainLoop Status: " + str(self.get_status_str())
      if self.is_ready():
        self.start_action_chain()
      time.sleep(ACTION_CHAIN_LAUNCHER_PAUSE)

  def init(self):
    self.set_status_if_in_one_of(manager_status_enum.idle, (manager_status_enum.before,))
    try:
      self._start_action_chain_when_appropriate()
    except rospy.exceptions.ROSInterruptException:
      print("Shutting down")
    if LOG:
      print "=== Completed Int manager loop ====="
  
  
#plan = ("action:doactivity2bold_intro_intronau_introstep_medium_low","action:progressprocstep1_introstep_preprocedure","action:goal")
#im = int_manager(dummy_ros_proxy(plan))
#im.init()
if __name__ == '__main__':
  im = int_manager(ros_proxy())
  rospy.init_node('ros_int_manager', anonymous=True)
  im.init()




