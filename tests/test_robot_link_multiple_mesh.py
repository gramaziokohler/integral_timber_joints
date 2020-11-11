import json

import compas
from compas.datastructures import Mesh
from compas.geometry import Box, Frame
from compas.robots import RobotModel
from compas.utilities import DataEncoder

mesh = Mesh.from_shape(Box(Frame.worldXY(), 5, 1, 3))

robot_model = RobotModel('rob')
robot_model.add_link('root_link', mesh)
dump = json.dumps(robot_model, cls=DataEncoder)
print (dump)

robot_model = RobotModel('rob')
robot_model.add_link('root_link', [mesh, mesh.copy()])
dump = json.dumps(robot_model, cls=DataEncoder)
print (dump)
