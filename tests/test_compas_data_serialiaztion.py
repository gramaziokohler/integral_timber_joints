from compas.data import Data
from compas.datastructures import Network
from integral_timber_joints.process import Movement
from compas.utilities import DataDecoder, DataEncoder
import json

# This test file make sure the Movement class can be serialized. Because it has some dict keys that are tuples.

m = Movement()
m.state_diff[('a', 'b')] = 1
str = json.dumps(m, cls=DataEncoder, indent=2, sort_keys=True)
print(str)

mm = json.loads(str, cls=DataDecoder)
print(mm.state_diff)
