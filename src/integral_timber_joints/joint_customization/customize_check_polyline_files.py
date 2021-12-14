import rhinoscriptsyntax as rs
from compas.geometry import Frame, Point, Vector, Transformation, distance_point_point
from compas_rhino.geometry import RhinoPoint
from integral_timber_joints.joint_customization.customize_polyline_lap import read_joint_customization_string, read_structural_model_transformation
from integral_timber_joints.joint_customization.customize_polyline_lap import find_joint_by_centroid
import logging
import os
from compas_rhino.utilities import unload_modules
unload_modules ("find_joint_by_centroid")
# joints_string_folder = "C:\\Users\\leungp\\Documents\\GitHub\\integral_timber_joints\\external\\itj_design_study\\211010_CantiBox\\joints\\left\\"
joints_string_folder = "C:\\Users\\leungp\\Documents\\GitHub\\integral_timber_joints\\external\\itj_design_study\\211010_CantiBox\\joints\\mid\\"
# joints_string_folder = "C:\\Users\\leungp\\Documents\\GitHub\\integral_timber_joints\\external\\itj_design_study\\211010_CantiBox\\joints\\right\\"

# This file help identify problems with individual polyline definitions.

print("Checking Polyline: %s" % joints_string_folder)


from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointPolylineLap

process = get_process()
artist = get_process_artist()
assembly = process.assembly

transformation = read_structural_model_transformation()

files = []
for i in range (36):
    files.append(joints_string_folder + str(i) + ".txt")



def find_joint_by_centroid(assembly, centroid_pt, distance_threshold = 50, verbose = True):

    for joint_id in assembly.joint_ids():
        joint = assembly.joint(joint_id)
        if joint.__class__.__name__ != "JointPolylineLap":
            continue
        distance = distance_point_point(joint.centroid, centroid_pt)
        if distance < distance_threshold:
            if verbose: print("Joint Found: %s, %s" % (joint_id, distance))
            return joint_id
    if verbose: print("Joint Not Found")
    return None

# files = files[0:6]
# files = files[6:12]
# files = files[12:18]
# files = files[18:24]
# files = files[24:30]
# files = files[30:36]

# files = [files[3]]

for file_name in files:
    file_path = joints_string_folder + file_name
    print("file_name: %s" %file_name)
    pt, v1, v2, v3, v4, polyline_string = read_joint_customization_string(file_name, transformation)
    # print("polyline_string = %s" % polyline_string)
    joint_id = find_joint_by_centroid(assembly, pt, distance_threshold=100, verbose = True)

    if joint_id is None:
        print ("Cannot find joint")
    else:
        height, polyline = eval(polyline_string)
        for i in range(4):
            if polyline[i][0] != [0.0, 0.0]:
                print ("Polyline %i start not at 0,0: %s" % (i, polyline[i][0]))
            if polyline[i][-1] != [1.0, 0.0]:
                print ("Polyline %i end not at 1,0: %s" % (i, polyline[i][0]))