import json
import os

from compas.datastructures.mesh.triangulation import mesh_quads_to_triangles
from compas.utilities import DataDecoder, DataEncoder

from integral_timber_joints.process import RobotClampAssemblyProcess


def hotfix_triangulate_tools(json_path):
    with open(json_path, 'r') as f:
        process = json.load(f, cls=DataDecoder)  # type: RobotClampAssemblyProcess

    print("Process loaded from: %s (%i Beams)." % (os.path.basename(json_path), len(list(process.assembly.beams()))))

    for tool in process.tools + [process.robot_toolchanger] + [process.robot_wrist]:
        print(tool)
        for link in tool.links:
            print(" - %s" % link.name)
            for visual in link.visual:
                mesh = visual.geometry.geo
                number_of_faces = mesh.number_of_faces()
                mesh_quads_to_triangles(mesh)
                print(" - - Face# from %s -> %s" % (number_of_faces, mesh.number_of_faces()))

    print("Environment")
    for key, mesh in process.environment_models.items():
        print("- %s" % key)
        number_of_faces = mesh.number_of_faces()
        mesh_quads_to_triangles(mesh)
        print(" - - Face# from %s -> %s" % (number_of_faces, mesh.number_of_faces()))

    with open(json_path, 'w') as f:
        json.dump(process, f, cls=DataEncoder, indent=None, sort_keys=True)


json_path = "C:\\Users\\leungp\\Documents\\GitHub\\integral_timber_joints\\external\\itj_design_study\\210128_RemodelFredPavilion\\twelve_pieces_process.json"
hotfix_triangulate_tools(json_path)

json_path = "C:\\Users\\leungp\\Documents\\GitHub\\integral_timber_joints\\external\\itj_design_study\\210128_RemodelFredPavilion\\pavilion_process.json"
hotfix_triangulate_tools(json_path)
