import compas
import itertools

from compas.datastructures import Mesh

robot_model = RobotModel('ro')
robot_model.add_link ('a', Mesh.from_obj(compas.get('boxes.obj')))
robot_model.add_link ('b', Mesh.from_obj(compas.get('boxes.obj')))
robot_model.add_joint('a','b')

def()
for link in robot_model.links:
    for
for link in self.links:
    for element in itertools.chain(link.collision, link.visual):
        mesh = element.shape.geometry # type = Mesh
        mesh

