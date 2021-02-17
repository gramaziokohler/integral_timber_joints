from compas.geometry import Box, Circle, Cylinder, Frame, Plane, Vector
from compas.datastructures import Mesh
from compas.robots import Joint, RobotModel

model = RobotModel(name='MinimalRobotModel')

# Link 1 has Box
box_1 = Box(Frame([2, .5, .25], [1, 0, 0], [0, 1, 0]), 1, 2, .5)
box_2 = Box(Frame([2, 0, 4], [1, 0, 0], [0, 1, 0]), .5, 1, 7)
link1 = model.add_link(name='link1', visual_meshes=[box_1, box_2])
# Link 2 has Cylinder
cylinder = Cylinder(Circle(Plane([0, 0, 0], [0, 0, 1]), .5), 8)
link2 = model.add_link(name='link2', visual_meshes=[cylinder])
# Link 3 has Mesh
v, f = Box(Frame([0, .5, .25], [1, 0, 0], [0, 1, 0]), 1, 2, .5).to_vertices_and_faces()
mesh = Mesh.from_vertices_and_faces(v,f)
link3 = model.add_link(name='link3', visual_meshes=[mesh])

origin = Frame([0, 0, 7], [1, 0, 0], [0, 1, 0])
axis = Vector(1, 0, 0)
model.add_joint(
     name='joint1',
     type=Joint.CONTINUOUS,
     parent_link=link1,
     child_link=link2,
     origin=origin,
     axis=axis,
)
model.add_joint(
     name='joint2',
     type=Joint.CONTINUOUS,
     parent_link=link2,
     child_link=link3,
     origin=origin,
     axis=axis,
)


file = 'robot_model.xml'
model.to_urdf_file(file, prettify=True)