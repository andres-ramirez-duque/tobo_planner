
import dummy_messages


LOG=True



class timeouts_parser :
  def __init__(self):
    self.timeouts = []
    
  def parse_timeout(self, line):
    self.timeouts.append((line[1], int(line[2])))

  def __str__(self):
    return "\n".join(map(lambda x: str(x), self.timeouts))

class requests_parser :
  def __init__(self):
    self.requests = []
    
  def parse_request(self, line):
    self.requests.append((line[1], line[2]))

  def __str__(self):
    return "\n".join(map(lambda x: str(x), self.requests))

class parameter(object):
  def get_parameter(self, op, aparams, message):
    pass
class value_parameter(parameter):
  def __init__(self, v):
    self.v = v
  def get_parameter(self, op, aparams, message):
    return self.v
  def __str__(self):
    return str(self.v)
class action_parameter(parameter):
  def __init__(self, i):
    self.arg = i-1
  def get_parameter(self, op, aparams, message):
    return aparams[self.arg]
  def __str__(self):
    return "$" + str(self.arg)
class message(parameter):
  def get_parameter(self, op, aparams, message):
    return message
  def __str__(self):
    return "$M"
class message_parameter(parameter):
  def __init__(self, x):
    self.field = x
  def get_parameter(self, op, aparams, message):
    print "WARNING: espec_parser.py/message_parameter: get_parameter not implemented!!"
class parameter_list(parameter):
  def __init__(self, peas):
    self.params = peas
  def get_parameter(self, op, aparams, message):
    l=[]
    for p in self.params:
      l.append(p.get_parameter(op, aparams, message))
    return l
  def __str__(self):
    return ";".join(map(lambda p: str(p), self.params))
class parameter_conc(parameter):
  def __init__(self, tok, peas):
    self.tok = tok
    self.params = peas
  def get_parameter(self, op, aparams, message):
    l = map(lambda p: p.get_parameter(op, aparams, message), self.params)
    return self.tok.join(l)
  def __str__(self):
    return self.tok.join(map(lambda p: str(p), self.params))
    
class effect(object):
  def __init__(self, when, op_pattern, effector, effect_type, parameters):
    self.when = when
    self.op_pattern = op_pattern
    self.effector = effector
    self.effect_type = effect_type
    self.parameters = parameters
  
  def __str__(self):
    l = self.when, self.op_pattern, self.effector, self.effect_type, map(lambda x: str(x), self.parameters)
    return "Effect:\n"+"\n".join(map(lambda x: str(x), l))

class parameter_parser:
  def parse_parameter(self, pstr):
    if pstr.startswith(":list:"):
      param = self.parse_lst(pstr)
    elif pstr.startswith(":conc:"):
      param = self.parse_conc(pstr)
    else:
      param = self.parse_exp(pstr)
    return param
    
  def parse_lst(self, pstr):
    c = []
    for e in pstr[6:].split(";"):
      c.append(self.parse_exp(e))
    return parameter_list(c)

  def parse_conc(self, pstr):
    c = []
    tok, cstr = pstr[6:].split(":",1)
    if tok == "sp": tok = " "
    for e in cstr.split(";"):
      c.append(self.parse_exp(e))
    return parameter_conc(tok, c)

  def parse_exp(self, pstr):
    if pstr.startswith(":int:"):
      return value_parameter(int(pstr[5:]))
    if pstr.startswith(":bool:"):
      return value_parameter(pstr[6]== "T")
    if pstr.startswith("$M"):
      if "." in pstr:
        print "WARNING: espec_parser.py/parse_exp: message parameter not implemented!!"
        sys.exit(1)
      else:
        return message()
    if pstr.startswith("$"):
      return action_parameter(int (pstr[1:]))
    else:
      return value_parameter(pstr)

class effects_parser :
  def __init__(self):
    self.effects = []
    self.pparser = parameter_parser()
    
  def parse_effect(self, line):
    when,op_pattern,effector,effect_type  = line[1:5]
    params = self.parse_params(line[5:])
    self.effects.append(effect(when,op_pattern,effector,effect_type, params))

  def parse_params(self, param_strs):
    params=[]
    for pstr in param_strs:
      param = self.pparser.parse_parameter(pstr)
      params.append(param)
    return params

  def __str__(self):
    return "\n".join(map(lambda x: str(x), self.effects))

class str_message :
  def __init__(self, p):
    self.p=p
    
  def make(self, op, params, indx):
    return self.p.get_parameter(op, params, None)

  def __str__(self):
    return str(self.p)

class web_message :
  def __init__(self, message_f, t_f):
    self.message_f=message_f
    self.t_f=t_f
    
  def make(self, op, params, indx):
    message = self.message_f.get_parameter(op, params, None)
    t = self.t_f.get_parameter(op, params, None)
    return dummy_messages.dummy_web_server_message(message, t, indx)

  def __str__(self):
    return str(self.message_f) + ", " + str(self.t_f)

class defaults_parser :
  def __init__(self):
    self.defaults = []
    self.pparser = parameter_parser()
  
  def parse_default(self, line):  
    provider,op_pattern,mtype = line[1:4]
    params = line[4:]
    message = self.parse_message(mtype, params)
    self.defaults.append((provider,op_pattern,message))
  
  def make_str_message(self, pstr):
    return str_message(self.pparser.parse_parameter(pstr))
    
  def make_web_message(self, params):
    args = map(lambda x: self.pparser.parse_parameter(x), params)
    return apply(web_message, args)
    
  def parse_message(self, mtype, params):
    fm = {"String": self.make_str_message, "Web": self.make_web_message}
    return fm[mtype](params)
  def __str__(self):
    return "\n".join(map(lambda x: str(x[0]) + ", " + str(x[1]) + ", " + str(x[2]), self.defaults))

def get_lines(fn):
  return map(lambda x: x.split(" "), filter(lambda x: not x == "", map(lambda x: x.split("#")[0].strip(), open(fn).readlines())))


def parse(fn):
  tp = timeouts_parser()
  rp = requests_parser()
  ep = effects_parser()
  dp = defaults_parser()
  
  parsers = {":timeout": tp.parse_timeout,
           ":request": rp.parse_request,
           ":effect": ep.parse_effect,
           ":default": dp.parse_default
          }

  lines = get_lines(fn)
  for line in lines:
    parsers[line[0]](line)
  if LOG:
    print tp
    print "--"
    print rp
    print "--"
    print ep
    print "--"
    print dp
  return tp.timeouts, rp.requests, ep.effects, dp.defaults






if __name__ == '__main__':
  import sys
  fn=sys.argv[1]
  parse(fn)


