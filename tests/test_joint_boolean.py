from integral_timber_joints.geometry import Joint_halflap, Beam
from integral_timber_joints.assembly import Assembly

from compas.geometry import Frame

joint = Joint_halflap(1, 100, 45, 50, 50, 50)
print(joint.data)

a = Assembly()


beam1 = Beam(name = '1')
beam2 = Beam(name = '2')
a.add_beam(beam1)
a.add_beam(beam2)
a.add_one_joint(joint,'1', '2')
mesh = a.get_beam_mesh_in_wcf('1')

print(mesh)
