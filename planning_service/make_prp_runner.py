
import os


def make_prp_runner(prp_root, solution_fn, deadend_detection, cmplan_abs_path, is_costed, use_local_search):
  
  
  mode = ""
  if is_costed: mode="costed "
  
  
  
  added_args_str = " --jic-limit 30 --plan-with-policy 0"
  if not deadend_detection:
    added_args_str += " --detect-deadends 0 --generalize-deadends 0 --online-deadends 0"
  if not use_local_search:
    added_args_str += " --partial-planlocal 0 --limit-planlocal 0 --planlocal 0"
  
  s = "#! /bin/bash\n\nif [[ $# < 2 ]]; then\n echo \"usage: "+ cmplan_abs_path +" <domain> <problem> [--keep-files]\"\nexit 1\nfi\n\n"
  s += prp_root + "/src/prp " + mode + "$1 $2 --dump-policy 2 --optimize-final-policy 1" + added_args_str + "\n\n"
  s += "export PYTHONPATH=${PYTHONPATH}:" + prp_root + "/prp-scripts\n"
  s += "python2 " + prp_root + "/prp-scripts/translate_policy.py > " + solution_fn + "\n\n"
  s += "if [ \"--keep-files\" != \"$3\" ]; then\n " +  prp_root + "/src/cleanup\n  rm -f policy.out\n  rm -f policy.fsap\n  rm -f " + solution_fn + "\nfi\n\n"
  
  open(cmplan_abs_path,'w').write(s)


if __name__ == '__main__':
  prp_root, solution_fn, deadend_detection = os.sys.argv[1:]
  make_prp_runner(prp_root, solution_fn, deadend_detection.lower() == "false")
  
  
  
