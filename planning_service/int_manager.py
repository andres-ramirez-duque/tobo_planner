#!/usr/bin/env python
import logging, sys
import threading
import time
import oyaml as yaml



LOG=True
SENSE_THEN_VALIDATE=True
ROS=False

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
  
def reconstruct_action_str(op, params):
  if len(params) == 0:
    return op
  return op+"_"+"_".join(params)

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


class dummy_planner_chain_message(object):
  def __init__(self, op, params, index):
    self.action_type=op
    self.parameters=params
    self.plan_step=index
  def __str__(self):
    return "DPCM [" + str(self.plan_step) + "] " + str(self.action_type) + "_" + "_".join(self.parameters)
  
class dummy_web_server_message(object):
  def __init__(self, message, t, indx):
    self.parameters = (message,)
    self.request_type = t
    self.plan_step = indx
  def __str__(self):
    return "DWSM [" + str(self.plan_step) + "] " + str(self.request_type) + " " + str(self.parameters)

class dummy_sensor_message(object):
  def __init__(self, message, t, indx):
    self.parameters = (message,)
    self.request_type = t
    self.plan_step = indx
  def __str__(self):
    return "DSRM [" + str(self.plan_step) + "] " + str(self.request_type) + " " + str(self.parameters)

######################################################################################################
### Threaded util classes ############################################################################
######################################################################################################

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

class ThreadedRequest(object):
  
  def make_request(self, args):
    x = threading.Thread(target=self.thread_function, args=args)
    x.start()
    
  def thread_function(self, args):
    if LOG:
      print str(args), "---> calling function..."
    self._make_request(args)
    
  def _make_request(self, args):
    pass

class ServiceRequest(ThreadedRequest):
  def __init__(self):
    super(ServiceRequest, self).__init__()
  
  def _make_request(self, args):
    self.make_service_request(args)

  def make_service_request(self, args):
    service, msg, msg_type = args
    rospy.wait_for_service(service, timeout=10)
    resp = None
    try:
      sensor_querier = rospy.ServiceProxy(service, msg_type)
      resp = sensor_querier(msg.request_type,msg.plan_step)
      self.service_response(resp, args)
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)
    

  def service_response(self, resp, args):
    pass

######################################################################################################
### ros proxies/dummies for planner, webserver and sensors ###########################################
######################################################################################################

class DummyWebServer(Timer):
  def __init__(self, t, s):
    super(DummyWebServer, self).__init__(t)
    self.web_server_status = web_server_status_enum.idle
    self.s=s
  
  def ask_for_user_input(self, options, default, timeout=10, args = (1,)): ## XXX timeout ignored here -could just set self.t?
    obj, label, indx = key_deconstruct(args[0])
    message_out = dummy_web_server_message(default, label, indx)
    self.start((message_out,))
    
  def request_change(self, params, args):
    obj, label, indx = key_deconstruct(args[0])
    message_out = dummy_web_server_message(None, label, indx)
    print "Making web server request: " + str(params)  
    
  def _trigger(self, message):
    apply(self.s, (message,))


class WebServer(object):
  def __init__(self, s):
    self.web_chain_sub = rospy.Subscriber("/request", web_chain, s) 
    self.web_chain_pub = rospy.Publisher('/listener_req', web_chain, queue_size=10)
    self.web_chain_command = rospy.Publisher('/listener_com', web_chain, queue_size=10)
  
  def ask_for_user_input(self, options, default, timeout=10, args = (1,)):
    obj, label, indx = key_deconstruct(args[0])
    msg = web_chain()
    msg.plan_step = indx
    msg.request_type = label
    msg.parameters = options
    msg.duration = timeout
    self.web_chain_pub.publish(msg)
    
  def request_change(self, params, args):
    obj, label, indx = key_deconstruct(args[0])
    msg = web_chain()
    msg.plan_step = indx
    msg.request_type = label
    msg.parameters = params
    msg.duration = -1
    self.web_chain_command.publish(msg)    
    

class DummyPlanner(Timer):
  def __init__(self, t, s, steps):
    super(DummyPlanner, self).__init__(t)
    self.s=s
    self.steps = steps
  
  def get_action(self, plan_step):
    action_bits = self.steps[plan_step].split("_")
    op = action_bits[0]
    params = action_bits[1:]
    message_out = dummy_planner_chain_message(op, params, plan_step)
    super(DummyPlanner, self).start((message_out,))
    
  def _trigger(self, args):
    apply(self.s, (args,)) 

class Planner(object):
  def __init__(self, s):
    self.action_sub = rospy.Subscriber("/next_action", action_chain, s)
    
  def get_action(self, plan_step):
    mode = "Get an action"
    print "%%%%%%%%%%%%% PASSING INTO PLANNER", mode, plan_step
    self.get_an_action_client(mode, plan_step)

  def get_an_action_client(self, planner_mode,plan_step):
    rospy.wait_for_service('get_an_action')
    try:
      get_an_action = rospy.ServiceProxy('get_an_action', PlannerMode)
      resp = get_an_action(planner_mode,plan_step)
      return resp.planner_ok
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)


class DummySensors(Timer):
  def __init__(self, t, s):
    super(DummySensors, self).__init__(t)
    self.s=s
  
  def get_sensor_value(self, default, args):
    obj, label, indx = key_deconstruct(args[0])
    message_out = dummy_sensor_message(default, label, indx)
    #super(DummySensors, self).start((message_out,))
    self.start((message_out,))
    
  def _trigger(self, args):
    apply(self.s, (args,))

class SensorRequest(ServiceRequest):
  def __init__(self, s):
    self.s=s
  
  def service_response(self, resp, args):
    _, msg, _ = args
    message = dummy_sensor_message(resp.sensor_value, msg.request_type, msg.plan_step)
    apply(self.s, (message,))

class Sensors(object):

  def __init__(self, s):
    self.s=s

  def get_sensor_value(self, default, args):
    obj, label, indx = key_deconstruct(args[0])
    msg = SensorValue()
    msg.plan_step = indx
    msg.request_type = label
    rargs = "get_sensor_value", msg, SensorValue
    sr = SensorRequest(self.s)    
    sr.make_request((rargs,))

######################################################################################################
### ros proxies/dummies entry point ##################################################################
######################################################################################################

class service_provider(object):
  def __init__(self):
    self._last_tag = None

  def initialise(self, action_broadcast_f, webserver_broadcast_f, sensors_broadcast_f,
                       nau_broadcast_f, stop_f):
    self.init_planner(action_broadcast_f)
    self.init_webserver(webserver_broadcast_f)
    self.init_sensors(sensors_broadcast_f)
    self.record_nau_listener(nau_broadcast_f)
    self.record_stop_listener(stop_f)
    
  def request_action(self, plan_index):
    self._request_action(plan_index)
  def ask_for_user_input(self, options, default, timeout, k):
    self._ask_for_user_input(options, default, timeout, k)
  def request_change(self, params, k):
    self.web_server.request_change(params, (k,))
  def sense_value(self, default, k):
    self._sense_value(default, k)  
  def _request_action(self, plan_index):
    self.planner.get_action(plan_index)
  def _ask_for_user_input(self, options, default, timeout, k):
    self.web_server.ask_for_user_input(options, default, timeout, (k,))
  def _sense_value(self, default, k):
    self.sensor_server.get_sensor_value(default, (k,))
  def set_parameter(self, path, v):
    print "[SP] Set: " + path + " to: " + str(v)
  def set_last_executed_action(self, a):
    pass
  def stop(self, req):
    pass
  def is_shutdown(self):
    pass

class dummy_ros_proxy(service_provider):
  def __init__(self, plan):
    super(dummy_ros_proxy, self).__init__()
    self.plan=plan
    
  def init_planner(self, action_broadcast_f):
    self.planner = DummyPlanner(2, action_broadcast_f, self.plan)
  def init_webserver(self, webserver_broadcast_f):
    self.web_server = DummyWebServer(10, webserver_broadcast_f)
  def init_sensors(self, sensors_broadcast_f):
    self.sensor_server = DummySensors(1, sensors_broadcast_f)
  def record_nau_listener(self, nau_broadcast_f):
    pass
  def record_stop_listener(self, stop_f):
    pass
  def is_shutdown(self):
    return False

class ros_proxy(service_provider):
  def __init__(self):
    super(ros_proxy, self).__init__()
    
  def init_planner(self, action_broadcast_f):
    self.planner = Planner(action_broadcast_f)
  def init_webserver(self, webserver_broadcast_f):
    self.web_server = WebServer(webserver_broadcast_f)
  def init_sensors(self, sensors_broadcast_f):
    self.sensor_server = Sensors(sensors_broadcast_f)
  def record_nau_listener(self, nau_broadcast_f):
    rospy.Subscriber("/naoqi_driver/ALAnimatedSpeech/EndOfAnimatedSpeech", action_chain, nau_broadcast_f)
  def record_stop_listener(self, stop_f):
    rospy.Service('/stop_nao', PlannerMode, stop_f)
  def set_parameter(self, path, v):
    rospy.set_param(path, v)
  def set_last_executed_action(self, a):
    path_to_stage_param="/parameters/last_executed_action" # maybe just a yaml parameter?
    rospy.set_param(path_to_stage_param, a)
  def stop(self, req):
    rospy.wait_for_service('/naoqi_driver/set_behavior')
    try:
      stop_action = rospy.ServiceProxy('/naoqi_driver/set_behavior', SetString)
      resp = stop_action(SetString('stopAllBehaviors'))
      print("Stoping NAO Behaviors")
      return resp.success
      try:
        sys.exit("Exit by StopAllBehaviors!")
      except SystemExit as message:
        print(message)
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)
  def is_shutdown(self):
    return rospy.is_shutdown()

######################################################################################################
### stats ############################################################################################
######################################################################################################


def get_time():
  try:
    t=time.perf_counter()
  except:
    t=time.time()
  return t

def output_stats(proc_duration, action_sequence, action_hierarchy, timing_bank):
  print "Procedure duration: " + str(round(proc_duration, 2))
  print "Total number of actions: " + str(len(action_sequence))
  robot = sensing = proc_sensing = delayers = enders = others = 0
  for (op,params) in action_sequence:
    h = action_hierarchy[op]
    labelled = False
    if "doactivity" in h:
      labelled=True
      robot+=1
    if "anxietytest" in h or "qtypepreference" in h or "engagementtest" in h:
      labelled=True
      sensing +=1
    if "firstcompleteprocedure" in h or "secondcompleteprocedure" in h or "querysitecheck" in h or "waitforproceduretoend" in h:
      labelled=True
      proc_sensing +=1
    if "wait" in h or "pause" in h:
      labelled=True
      delayers += 1
    if "goal" in h:
      labelled=True
      enders += 1
    if not labelled: 
      others += 1
    
  print "-- Number of robot behaviours: " + str(robot)
  print "-- Number of sensing actions: " + str(sensing)
  print "-- Number of procedure step query: " + str(proc_sensing)
  print "-- Number of delaying actions: " + str(delayers)
  print "-- Number of other actions: " + str(others)
  print "-- Number of ender actions: " + str(enders)
  print
  print "Actions sequence: " + "; ".join(map(lambda (op,params): "(" + op + " " +" ".join(params)+")", action_sequence))
  print 
  print "Additional timers:"
  for label in timing_bank:
    print "-- Timer for " + str(label) + ": " + str(round(timing_bank[label], 2))
  
  
######################################################################################################
### interaction manager ##############################################################################
######################################################################################################

class int_manager(object):
  def __init__(self, service_provider, sffn):
    self.service_provider = service_provider
    self.service_provider.initialise(self.planner_message_event, self.webserver_message_event,
                                     self.sensor_message_event, self.nau_finish_message_event,
                                     self.stop_message_event)
    self.active_requests = []
    self.request_defaults = {}
    self.active_requests_lock = threading.Lock()
    
    self._action_sequence=[]
    self._timing_bank = {}
    self._open_timers = {}
    
    self.status = manager_status_enum.before
    self.status_lock = threading.Lock()
    self.procedure_status = procedure_status_enum.null # XXX TODO
    self.interaction_step_status = None # XXX TODO
    
    self.counter=-1
    self.parse_state_frame(sffn)
    
  def parse_state_frame(self, sffn):
    with open(sffn, "r") as stream:
      frame_dict=yaml.safe_load(stream)
    self._action_hierarchy = dict(map(lambda (k,v): (k,v.split(",")), frame_dict["action_hierarchy"].items()))
    self._op_timeout = dict(map(lambda (k,v): (k,int(v)), frame_dict["timeouts"].items()))
    self._bool_parameters = {}
    if "parameters" in frame_dict:
      params = frame_dict["parameters"]
      if "boolean_vars" in params:
        self._bool_parameters = params["boolean_vars"]
    self._bool_sensors = {}
    if "sensors" in frame_dict:
      params = frame_dict["sensors"]
      if "boolean_vars" in params:
        self._bool_sensors = params["boolean_vars"]

  ######################################################################################################
  ### action early implementation ######################################################################
  ######################################################################################################

  def tell_webserver_progress_proc_step(self, stage):
    self.service_provider.request_change((stage,), key_maker("web server", "stage progression", self.counter))

  def process_stage_changes(self, op, params):
    if "introduction" in self._action_hierarchy[op]:
      self.tell_webserver_progress_proc_step("introduction")
    elif "startpreprocedure" in self._action_hierarchy[op]:
      self.tell_webserver_progress_proc_step("preprocedure")
    elif "startsitecheck" in self._action_hierarchy[op]:
      self.tell_webserver_progress_proc_step("sitecheck")
    elif "startprocedure" in self._action_hierarchy[op]:
      self.tell_webserver_progress_proc_step("procedure")
    elif "debrief" in self._action_hierarchy[op]:
      self.tell_webserver_progress_proc_step("debrief")
    elif "finish" in self._action_hierarchy[op]:
      self.tell_webserver_progress_proc_step("finish")
    elif "goal" in self._action_hierarchy[op]:
      self.tell_webserver_progress_proc_step("goal")
    
  def process_wait(self, op, params):
    label = "wait"
    options = ("Ready")
    default="Defaulted"
    self.service_provider.ask_for_user_input(options, default, -1, key_maker("web server",label, self.counter))
    self.add_flag(label, default)
    self._open_timers[label]=get_time()

  def process_type_preference_query(self, op, params, t):
    label = "type preference query"
    options = ("calm","active")
    default="active"
    self.service_provider.ask_for_user_input(options, default, t, key_maker("web server",label, self.counter))
    self.add_flag(label, default)
    self._open_timers[label]=get_time()

  def process_activity_preference_query(self, op, params, t):
    label = "activity preference query"
    s1,s2 = params[0:2]
    default=s1
    self.service_provider.ask_for_user_input((s1,s2), default, t, key_maker("web server",label, self.counter))
    self.add_flag(label, default)
    self._open_timers[label]=get_time()
    
  def process_engagement_test(self, op, params, t):
    label = "engagement test"
    options = ("true","false")
    default="true"
    self.service_provider.ask_for_user_input(options, default, t, key_maker("web server",label, self.counter))
    self.add_flag(label, default)
    self._open_timers[label]=get_time()

  def process_engagement_validate(self, op, params, t, default="true"):
    label = "engagement validate"
    options = ("true","false")
    #default="true"
    self.service_provider.ask_for_user_input(options, default, t, key_maker("web server",label, self.counter))
    #self.add_flag(label, default) XXX we have lock
    self.active_requests.append(label)
    self.request_defaults[label]=default
    self._open_timers[label]=get_time()

  def request_engagement_sensor_value(self, op, params):
    label = "engagement sensed"
    default="true"
    self.service_provider.sense_value(default, key_maker("sensors", "engagement sensed", self.counter))
    self.add_flag(label, default)
    self._open_timers[label]=get_time()

  def process_anxiety_test(self, op, params, t):
    label = "anxiety test"
    options = ("true","false")
    default="true"
    self.service_provider.ask_for_user_input(options, default, t, key_maker("web server",label, self.counter))
    self.add_flag(label, default)
    self._open_timers[label]=get_time()

  def process_procedure_complete_query(self, op, params, t):
    label = "procedure complete query"
    options = ("true","false")
    default="true"
    self.service_provider.ask_for_user_input(options, default, t, key_maker("web server",label, self.counter))
    self.add_flag(label, default)
    self._open_timers[label]=get_time()

  def process_site_check_query(self, op, params): 
    label = "site check query"
    options = ("true","false")
    default="true"
    self.service_provider.ask_for_user_input(options, default, -1, key_maker("web server",label, self.counter))
    self.add_flag(label, default)
    self._open_timers[label]=get_time()

  def process_wait_procedure_end(self, op, params): 
    label = "procedure ended ok query"
    options = ("true","false")
    default="true"
    self.service_provider.ask_for_user_input(options, default, -1, key_maker("web server",label, self.counter))
    self.add_flag(label, default)
    self._open_timers[label]=get_time()

  def process_do_activity_action(self, op, params):
    label = "nau behaviour"
    self.add_flag(label, None)
    self._open_timers[label]=get_time()

  def process_action_execution(self, op, params):
    self._action_sequence.append((op,params))
    self._current_action = reconstruct_action_str(op, params)
    self.set_status_if_in_one_of(manager_status_enum.executing, (manager_status_enum.planning,))
    timeout_label = None
    
    self.process_stage_changes(op, params)
    
    if "doactivity" in self._action_hierarchy[op]:
      self.process_do_activity_action(op, params)
      timeout_label = "doactivity"
    elif "idle" in self._action_hierarchy[op]:
      timeout_label = "idle"
    elif "ivquerysitecheck" in self._action_hierarchy[op]:
      timeout_label = "wait"
      self.process_site_check_query(op, params)
    elif "waitforproceduretoend" in self._action_hierarchy[op]:
      timeout_label = "wait"
      self.process_wait_procedure_end(op, params)
    elif "wait" in self._action_hierarchy[op]:
      self.process_wait(op, params)
      timeout_label = "wait"
    elif "pause" in self._action_hierarchy[op]:
      timeout_label = "pause"
    elif "anxietytest" in self._action_hierarchy[op]:
      timeout_label = "query_response"
      self.process_anxiety_test(op, params, self._op_timeout[timeout_label])
    elif "qactivitypreference" in self._action_hierarchy[op]:
      timeout_label = "query_response"
      self.process_activity_preference_query(op, params, self._op_timeout[timeout_label])
    elif "qtypepreference" in self._action_hierarchy[op]:
      timeout_label = "query_response"
      self.process_type_preference_query(op, params, self._op_timeout[timeout_label])
    elif "engagementtest" in self._action_hierarchy[op]:
      psym = "eamdisengaged"
      if psym in self._bool_parameters:
        timeout_label = "query_response"
        self.process_engagement_test(op, params, self._op_timeout[timeout_label])
      elif psym in self._bool_sensors:
        timeout_label = "query_response"
        self.request_engagement_sensor_value(op, params)  
    elif "firstcompleteprocedure" in self._action_hierarchy[op]:
      timeout_label = "query_response"
      self.process_procedure_complete_query(op, params, self._op_timeout[timeout_label])
    elif "secondcompleteprocedure" in self._action_hierarchy[op]:
      timeout_label = "query_response"
      self.process_procedure_complete_query(op, params, self._op_timeout[timeout_label])
    elif "goal" in self._action_hierarchy[op]:      
      if not self.set_status_if_in_one_of(manager_status_enum.after, (manager_status_enum.executing,)):
        print "WARNING: goal achieved, but manager lost.."
      return
    else:
      print "WARNING: unknown action: ", op, params
      timeout_label = "unknown_action"
    if timeout_label in self._op_timeout:
      MessageGiver(self._op_timeout[timeout_label], self.on_timer_event, None).start((key_maker("manager","timeout",self.counter),))
    

  ######################################################################################################
  ### action late implementation #######################################################################
  ######################################################################################################
  
  def if_bool_parameter_then_set(self, psym, v):
    if psym in self._bool_parameters:
      path_to_stage_param="/parameters/boolean_vars/" + psym
      self.service_provider.set_parameter(path_to_stage_param, v)
    elif psym in self._bool_sensors:
      path_to_stage_param="/sensors/boolean_vars/" + psym
      self.service_provider.set_parameter(path_to_stage_param, v)

  def update_timing_bank(self, label):
    if label in self._open_timers:
      if not label in self._timing_bank:
        self._timing_bank[label]=0
      self._timing_bank[label] += get_time() - self._open_timers[label]
      self._open_timers.pop(label)
  
  def process_request_reply(self, flag, message):
    self.update_timing_bank(flag)
  
    if flag == "nau behaviour":
      pass
    elif flag == "idle":
      pass
    elif flag == "wait":
      pass
    elif flag == "pause":
      pass
    elif flag == "anxiety test":
      self.if_bool_parameter_then_set("amanxietymanaged", message)
    elif flag == "engagement test":
      self.if_bool_parameter_then_set("eamdisengaged", not str(message).lower() == "true")
    elif flag == "engagement sensed":
      if SENSE_THEN_VALIDATE:
        op = "engagementtest"
        params = []
        timeout_label = "query_response"
        t = self._op_timeout[timeout_label]
        self.process_engagement_validate(op, params, t, default=str(message).lower())
        MessageGiver(t, self.on_timer_event, None).start((key_maker("manager","timeout", self.counter),))
      else:
        self.if_bool_parameter_then_set("eamdisengaged", not str(message).lower() == "true")
    elif flag == "engagement validate":
      self.if_bool_parameter_then_set("eamdisengaged", not str(message).lower() == "true")
    elif flag == "type preference query":
      self.if_bool_parameter_then_set("uselecteddivert", str(message).lower() == "active")
    elif flag == "activity preference query":
      self.if_bool_parameter_then_set("uselected " + str(message), True)
    elif flag == "site check query":
      self.if_bool_parameter_then_set("requiressitecheck", message)
    elif flag == "procedure ended ok query":
      self.if_bool_parameter_then_set("procedurehasfinished", message)
    elif flag == "procedure complete query":
      self.if_bool_parameter_then_set("procedurehasfinished", message)
    else:
      print "TODO: do something about ", flag, message
  
  

  ######################################################################################################
  ### locking util methods #############################################################################
  ######################################################################################################

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

  def force_set_status(self, status):
    self.status_lock.acquire()
    self.status = status
    self.status_lock.release()

  ######################################################################################################
  ### active request maintenance #######################################################################
  ######################################################################################################

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

  def record_if_requests_completed(self):
    self.active_requests_lock.acquire()
    self._record_if_requests_completed()
    self.active_requests_lock.release()
  def _record_if_requests_completed(self):
    if len(self.active_requests) == 0:
      self.service_provider.set_last_executed_action(self._current_action)
      self._current_action=None
      self.set_status_if_in_one_of(manager_status_enum.idle, (manager_status_enum.executing,))
  
  def handle_disengagements(self, flag):
    if flag == "stage progression":
      self.service_provider.disable_user_info_request(flag)

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

  def handle_timeout(self, indx, message):
    if self.status_is_one_of((manager_status_enum.executing,)):
      self.active_requests_lock.acquire()
      if self.counter == indx :
        if LOG:
          print "@TIMEOUT:"
        
        cp_active_requests = self.active_requests[:]
        for flag in cp_active_requests:
          self.process_request_reply(flag, self.request_defaults[flag])
          self.handle_disengagements(flag)
          self.active_requests.remove(flag)
          
        self._record_if_requests_completed()
      else:
        if LOG:
          print "Ignoring timeout - previous step.."
      self.active_requests_lock.release()
    else:
        if LOG:
          print "Ignoring timeout - no longer executing.."


  ######################################################################################################
  ### broadcast handlers ###############################################################################
  ######################################################################################################

  def sensor_message_event(self, sensors_message): # a reply from the sensors
    message = str(sensors_message.parameters[0])
    t = sensors_message.request_type
    indx = sensors_message.plan_step
    if self.remove_request_if_active(t, indx):
      if LOG:
        print "++++ Sensor message (" + str(t)+"): " + str(message)
      self.process_request_reply(t, message)
      self.record_if_requests_completed()
    else:
      if LOG:
        print "++++ Ignoring: ", str(sensors_message)

  def planner_message_event(self, action_message): # a broadcast from the planner
    self.update_timing_bank("planner")
    op = action_message.action_type 
    params = action_message.parameters
    indx = action_message.plan_step
    if LOG:
        print "++++ Planner response: ("+str(op)+ " " + " ".join(map(lambda x: str(x), params)) +")"
    self.process_action_execution(op, params)

  def webserver_message_event(self, web_message):
    message = str(web_message.parameters[0])
    t = web_message.request_type
    indx = web_message.plan_step
    
    if self.remove_request_if_active(t, indx):
      if LOG:
        print "++++ Web server message ("+str(t)+"): " + str(message)
      self.process_request_reply(t, message)
      self.record_if_requests_completed()
    else:
      if LOG:
        print "++++ Ignoring: ", str(web_message)

  def on_timer_event(self, message, k): # timeout event
    obj, t, indx = key_deconstruct(k)
    if obj == "manager":
      if t == "timeout":
        self.handle_timeout(indx, message)
      else:
        print "WARNING: unknown manager type: ", message, k

  def nau_finish_message_event(self, action_message): # a broadcast from nau
    action_str = reconstruct_action_str(action_message.action_type, action_message.parameters)
    t = "nau behaviour"
    indx = action_message.plan_step

    if self.remove_request_if_active(t, indx):
      if LOG:
        print "++++ On timer event: " + str(action_str) + " from: ", t, indx
      self.process_request_reply(t, action_message)
      self.record_if_requests_completed()
    else:
      if LOG:
        print "++++ Ignoring: ", t, indx
  
  def stop_message_event(self, req):
    self.service_provider.stop(req)
    self._stop()
  
  def _stop(self):
    self.force_set_status(manager_status_enum.after)

  ######################################################################################################
  ### main loop ########################################################################################
  ######################################################################################################

  def start_action_chain(self):
    self.incr_counter_and_clear_flags()

    if LOG:
      print "+ Calling planner..", self.counter
    if self.set_status_if_in_one_of(manager_status_enum.planning, (manager_status_enum.idle,)):
      self._open_timers["planner"]=get_time()
      self.service_provider.request_action(self.counter)

  def _start_action_chain_when_appropriate(self):
    while not self.is_finished() and not self.service_provider.is_shutdown():
      print "@@@@@@MainLoop Status: " + str(self.get_status_str())
      if self.is_ready():
        self.start_action_chain()
      time.sleep(self._op_timeout["action_chain_launcher_pause"])

  def init(self):
    start_time = get_time()
    self.set_status_if_in_one_of(manager_status_enum.idle, (manager_status_enum.before,))
    self._start_action_chain_when_appropriate()
    if LOG:
      print "=== Completed Int manager loop ====="
    end_time = get_time()
    proc_duration = end_time - start_time
    output_stats(proc_duration, self._action_sequence, self._action_hierarchy, self._timing_bank)

if __name__ == '__main__':
  import rospy

  from std_msgs.msg import String
  from tobo_planner.msg import action_chain
  from tobo_planner.msg import web_chain
  from tobo_planner.srv import PlannerMode,PlannerModeResponse,SensorValue,SensorValueResponse
  from naoqi_bridge_msgs.srv import SetString
  
  yaml_file = rospy.get_param("/state_frame_fn")
  im = int_manager(ros_proxy(), yaml_file)
  rospy.init_node('ros_int_manager')
  try:
    im.init()
  except rospy.exceptions.ROSInterruptException:
      print("Shutting down")



