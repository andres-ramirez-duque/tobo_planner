
def request_feature_value(name, accepted_values):
  print ("    Choice: " + str(name))
  ordered_options = list(accepted_values)
  for i, option in enumerate(accepted_values):
    print ("      Option [" + str(i) + "]: " + str(option))
  option_indexes=range(len(accepted_values))
  choice = int(input ("    >>> Select current state value from " + str(list(option_indexes))+": "))
  return ordered_options[choice]


def request_value(query):
  return input ("    Query: " + str(query) + ": ")

