
import os


if __name__ == '__main__':
  DEMO_ROOT=os.path.dirname(os.path.abspath(os.sys.argv[0]))
  prp_root, solution_fn, deadend_detection = os.sys.argv[1:]
  
  deadend_str = ""
  if deadend_detection.lower() == "false":
    deadend_str = " --detect-deadends 0 --generalize-deadends 0 --online-deadends 0"
  
  s = "#! /bin/bash\n\nif [[ $# < 2 ]]; then\n echo \"usage: ./cmplan <domain> <problem> [--keep-files]\"\nexit 1\nfi\n\n"
  s += prp_root + "/src/prp $1 $2 --dump-policy 2 --optimize-final-policy 1" + deadend_str + "\n\n"
  s += "export PYTHONPATH=${PYTHONPATH}:" + prp_root + "/prp-scripts\n"
  s += "python2 " + prp_root + "/prp-scripts/translate_policy.py > " + solution_fn + "\n\n"
  s += "if [ \"--keep-files\" != \"$3\" ]; then\n " +  prp_root + "/src/cleanup\n  rm -f policy.out\n  rm -f policy.fsap\n  rm -f " + solution_fn + "\nfi\n\n"
  
  
  open("cmplan",'w').write(s)
  
  
