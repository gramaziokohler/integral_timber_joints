import os, sys
from termcolor import cprint

HERE = os.path.abspath(os.path.dirname(__file__))
try:
    # prioritize local pddlstream first
    # add your PDDLStream path here: https://github.com/caelan/pddlstream
    sys.path.append(os.environ['PDDLSTREAM_PATH'])
except KeyError:
    cprint('No `PDDLSTREAM_PATH` found in the env variables, using pddlstream submodule', 'yellow')
    sys.path.extend([
       os.path.abspath(os.path.join(HERE, '..', '..', '..', 'external', 'pddlstream'))
    ])

try:
    sys.path.append(os.environ['PYPLANNERS_PATH'])
except KeyError:
    cprint('No `PYPLANNERS_PATH` found in the env variables, using pyplanner submodule', 'yellow')
    here = os.path.abspath(os.path.dirname(__file__))
    pyplanner_path = os.path.abspath(os.path.join(here, '..', '..', 'external', 'pyplanners'))
    # Inside PDDLStream, it will look for 'PYPLANNERS_PATH'
    os.environ['PYPLANNERS_PATH'] = pyplanner_path

import pddlstream
cprint("Using pddlstream from {}".format(pddlstream.__file__), 'yellow')

import strips # pyplanners
cprint("Using strips (pyplanners) from {}".format(strips.__file__), 'yellow')

