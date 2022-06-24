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

