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

import pddlstream
cprint("Using pddlstream from {}".format(pddlstream.__file__), 'yellow')

