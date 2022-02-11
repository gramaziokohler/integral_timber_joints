import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext as sc
import re
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointHalfLap, JointNonPlanarLap
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.tools import Clamp, Screwdriver, Gripper

import json

from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh
from compas.data import DataEncoder


from compas.geometry import Cylinder, Transformation


from compas.datastructures import Mesh, mesh_weld

if __name__ == '__main__':
    process = get_process()
    assembly = process.assembly
    artist = get_process_artist()
    scene = artist.get_current_selected_scene_state(artist)
    meshes_for_objects = artist._get_state_attached_objects_meshes(scene, attached_objects_only=True)
    for object_id in meshes_for_objects:
        object_state, meshes = meshes_for_objects[object_id]
        for mesh in meshes:
            welded_mesh = mesh_weld(mesh, 15)
            print (
                "Before:",
                len(list(mesh.vertices())),
                "After Weld:",
                len(list(welded_mesh.vertices()))
                )

            # print ("Before:")
            # for key in mesh.vertices():
            #     print (mesh.vertex_coordinates(key))
            # print ("After Weld:")
            # for key in welded_mesh.vertices():
            #     print (welded_mesh.vertex_coordinates(key))
            # break
        # break