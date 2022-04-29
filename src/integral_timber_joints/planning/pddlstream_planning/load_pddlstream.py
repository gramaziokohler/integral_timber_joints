import os, sys
from termcolor import cprint

HERE = os.path.abspath(os.path.dirname(__file__))
try:
    # prioritize local pddlstream first
    # add your PDDLStream path here: https://github.com/caelan/pddlstream
    sys.path.append(os.environ['PDDLSTREAM_PATH'])
except KeyError:
    pddlstream_path = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..', 'external', 'pddlstream'))
    sys.path.append(pddlstream_path)
    # cprint('No `PDDLSTREAM_PATH` found in the env variables', 'yellow')
    cprint('using pddlstream submodule {}'.format(pddlstream_path), 'yellow')

try:
    sys.path.append(os.environ['PYPLANNERS_PATH'])
except KeyError:
    pyplanner_path = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..', 'external', 'pyplanners'))
    # Inside PDDLStream, it will look for 'PYPLANNERS_PATH'
    os.environ['PYPLANNERS_PATH'] = pyplanner_path
    sys.path.append(pyplanner_path)
    # cprint('No `PYPLANNERS_PATH` found in the env variables, using pyplanner submodule', 'yellow')
    cprint('using pyplanner submodule {}'.format(pyplanner_path), 'yellow')

import pddlstream
cprint("Using pddlstream from {}".format(os.path.dirname(pddlstream.__file__)), 'yellow')

import strips # pyplanners
cprint("Using strips (pyplanners) from {}".format(os.path.dirname(strips.__file__)), 'yellow')
