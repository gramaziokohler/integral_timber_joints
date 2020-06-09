import jsonpickle
import pickle
jsonpickle.set_preferred_backend('simplejson')

from compas.geometry import Point
from integral_timber_joints.geometry import Beam, Joint_90lap
from integral_timber_joints.assembly import Assembly
from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.tools import Clamp

path_output_json = "tests/data/pickle_cpython.json"
path_output_pickle = "tests/data/pickle_cpython.pickle"

# data = Point(1,2,3)

beam1 = Beam.debug_get_dummy_beam()
beam2 = Beam.debug_get_dummy_beam()
beam1.name = 'b1'
beam2.name = 'b2'
joint1 = Joint_90lap(100, 3, 100, 100, 50, "dummy_joint")
joint2 = Joint_90lap(100, 3, 100, 100, 50, "dummy_joint")

assembly = Assembly()
assembly.add_beam(beam1)
assembly.add_beam(beam2)
assembly.add_joint_pair(joint1, joint2, 'b1', 'b2')
# assembly.update_beam_mesh_with_joints('b1')
# assembly.update_beam_mesh_with_joints('b2')
p = RobotClampAssemblyProcess(assembly)

# Load Tools
with open('C:/Users/leungp/Documents/GitHub/integral_timber_joints/examples/tools_example/PG1.json', 'r') as f:
    gripper = jsonpickle.decode(f.read(), keys=True)
    gripper.name = 'G1'
# p.add_gripper(gripper)
for link in gripper.links:
    print (link)

print (" --- ")
# Load Tools
with open('C:/Users/leungp/Documents/GitHub/integral_timber_joints/examples/tools_example/CL1.json', 'r') as f:
    clamp = jsonpickle.decode(f.read(), keys=True)
    clamp.name = 'C1'
# p.add_clamp(clamp) # This break things

for link in clamp.links:
    print (link)
# clamp = Clamp('c1', "SpecialClamp")

clamp = Clamp.Lap90ClampFactory(
    'c1',
    "SpecialClamp",
    1,
    2,
    3,
    4,
    tool_coordinate_frame = 5,
    tool_pick_up_frame = 6,
    tool_storage_frame = 7,
    mesh_gripper_base = 8,
    mesh_gripper_jaw_l = 9,
    mesh_gripper_jaw_r = 10,
    mesh_clamp_jaw_l = 11,
    mesh_clamp_jaw_r = 12,
    approach_vector = 13,
    detachretract1_vector = 14,
    detachretract2_vector = 15,
    )

for link in clamp.links:
    print (link)

data = clamp
with open(path_output_json, 'w') as f:
    f.write(jsonpickle.encode(data))

with open(path_output_pickle, 'wb') as f:
    pickle.dump(data,f, protocol=2)

