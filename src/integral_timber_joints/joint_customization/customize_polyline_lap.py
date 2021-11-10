import rhinoscriptsyntax as rs
from compas.geometry import Frame, Point, Vector, Transformation, distance_point_point
from compas_rhino.geometry import RhinoPoint

import logging
import os

logger = logging.getLogger('customize_polyline_lap')
joints_string_folder = "C:\\Users\\leungp\\Documents\\GitHub\\integral_timber_joints\\external\\itj_design_study\\211010_CantiBox\\joints\\left\\"

logging_level = logging.DEBUG
log_path = os.path.join(joints_string_folder, 'run.log')

logging.basicConfig(filename=log_path, format='%(asctime)s | %(name)s | %(levelname)s | %(message)s', level=logging_level)
logger.info(" -- ")
logger.info(" -- ")
logger.info("customize_polyline_lap started with folder: %s" % joints_string_folder)
print("customize_polyline_lap started with folder: %s" % joints_string_folder)

def read_structural_model_transformation():
    # Get 3 points by name as the origin for transformation
    model_origin = rs.coerce3dpoint(rs.ObjectsByName("model_origin")[0])
    model_x = rs.coerce3dpoint(rs.ObjectsByName("model_x")[0])
    model_y = rs.coerce3dpoint(rs.ObjectsByName("model_y")[0])

    model_origin = RhinoPoint.from_geometry(model_origin).to_compas()
    model_x = RhinoPoint.from_geometry(model_x).to_compas()
    model_y = RhinoPoint.from_geometry(model_y).to_compas()

    xaxis = Vector.from_start_end(model_origin, model_x)
    yaxis = Vector.from_start_end(model_origin, model_y)
    model_frame = Frame(model_origin, xaxis, yaxis)
    logger.debug("Structural model_frame: %s" % model_frame)

    T = Transformation.from_frame(model_frame)
    return T

def find_joint_by_centroid(assembly, centroid_pt, distance_threshold = 50):

    for joint_id in assembly.joint_ids():
        joint = assembly.joint(joint_id)
        if joint.__class__.__name__ != "JointPolylineLap":
            continue
        distance = distance_point_point(joint.centroid, centroid_pt)
        if distance < distance_threshold:
            print("Joint Found: %s, %s" % (joint_id, distance))
            logger.info("Joint Found: %s, %s" % (joint_id, distance))
            return joint_id
    print("Joint Not Found")
    logger.warning("Joint Not Found")
    return None

def change_joint_polyline (joint_id, polyline_string, v1_on_side, v2_on_side):


    # Set polyline
    joint = assembly.joint(joint_id) # type: JointPolylineLap
    joint_nbr = assembly.joint(reversed(joint_id))  # type: JointPolylineLap

    joint.param_string = polyline_string
    joint_nbr.param_string = polyline_string

    for _ in range(v1_on_side - 1):
        logger.info(" - rotate CCW")
        joint.polyline_rotate_ccw()
        joint_nbr.polyline_rotate_ccw()

    if (v2_on_side % 4 ) + 1 == v1_on_side:
        logger.info(" - flip 2-4")
        joint.polyline_flip_2_4()
        joint_nbr.polyline_flip_2_4()

    # TODO Flipping and turning

    polyline_lengths = [len(l) for l in joint.polylines]
    print ("polyline_lengths: %s" % polyline_lengths)


    # Update the interactive beam
    print ("Updating Beam %s (redraw_interactive_beam)" % joint_id[0])
    artist.redraw_interactive_beam(joint_id[0], redraw=False)
    print ("Updating Beam %s (redraw_interactive_beam)" % joint_id[1])
    artist.redraw_interactive_beam(joint_id[1], redraw=True)

def read_joint_customization_string(file_path, transformation):
    # Decode Input String
    with open(file_path, 'r') as f:
        input_string = f.read()
    # input_string = "[[-3.3315692002128099, 1.03251625305613, 2.4500000000000002], [-0.98480800000000002, 0.173648, 0.0], [0.33682400000000001, -0.059390999999999999, 0.939693], [0.98480800000000002, -0.173648, 0.0], [-0.33682400000000001, 0.059390999999999999, -0.939693], '[50.,[[[0.,0.],[0.1,0.],[0.11,0.1],[0.5,0.2],[1.,0.]],[[0.,0.],[0.5,0.15],[0.9,0.11],[1.,0.1],[1.,0.]],[[0.,0.],[0.1,0.],[0.11,0.1],[0.5,0.2],[1.,0.]],[[0.,0.],[0.5,0.15],[0.9,0.11],[1.,0.1],[1.,0.]]]]']"
    pt, v1, v2, v3, v4, polyline_string = eval(input_string)
    pt = Point(*[p * 1000 for p in pt]).transformed(transformation)
    v1 = Vector(*v1).transformed(transformation).unitized()
    v2 = Vector(*v2).transformed(transformation).unitized()
    v3 = Vector(*v3).transformed(transformation).unitized()
    v4 = Vector(*v4).transformed(transformation).unitized()

    return (pt, v1, v2, v3, v4, polyline_string)

def incoming_joint_on_side(assembly, joint_id, v1, v2):
    joint = assembly.joint(joint_id) # type: JointPolylineLap

    # * Find out the vector alignment
    side_1 = Vector.from_start_end(joint.corner_pts[3], joint.corner_pts[0]).unitized()
    side_2 = Vector.from_start_end(joint.corner_pts[0], joint.corner_pts[1]).unitized()
    side_3 = Vector.from_start_end(joint.corner_pts[0], joint.corner_pts[3]).unitized()
    side_4 = Vector.from_start_end(joint.corner_pts[1], joint.corner_pts[0]).unitized()

    v1_dot_products = [side_1.dot(v1), side_2.dot(v1), side_3.dot(v1), side_4.dot(v1)]
    logger.debug(v1_dot_products)
    v1_on_side = v1_dot_products.index(max(v1_dot_products)) + 1
    v2_dot_products = [side_1.dot(v2), side_2.dot(v2), side_3.dot(v2), side_4.dot(v2)]
    logger.debug(v2_dot_products)
    v2_on_side = v2_dot_products.index(max(v2_dot_products)) + 1
    logger.info("v1_on_side=%i v2_on_side=%i" % (v1_on_side, v2_on_side))
    if abs(v1_on_side - v2_on_side) not in [1,3]:
        logger.warning("v1_on_side=%i v2_on_side=%i" % (v1_on_side, v2_on_side))

    return (v1_on_side, v2_on_side)


from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointPolylineLap
process = get_process()
artist = get_process_artist()
assembly = process.assembly

transformation = read_structural_model_transformation()

# files =  [p for p in os.listdir(joints_string_folder) if p.endswith('txt')]
# files = files [0:3]
# files = [files[5]]
files = []
for i in range (36):
    files.append(joints_string_folder + str(i) + ".txt")

# files = files[0:6]
# files = files[6:12]
# files = files[12:18]
# files = files[18:24]
# files = files[24:30]
files = files[30:36]

# files = [files[23]]

for file_name in files:
    file_path = joints_string_folder + file_name
    logger.info("File: %s" %file_path)
    print("file_name: %s" %file_name)
    pt, v1, v2, v3, v4, polyline_string = read_joint_customization_string(file_name, transformation)
    logger.info("polyline_string = %s" % polyline_string)
    print("polyline_string = %s" % polyline_string)
    joint_id = find_joint_by_centroid(assembly, pt)

    if joint_id is not None:

        v1_on_side, v2_on_side = incoming_joint_on_side(assembly, joint_id, v1, v2)

        change_joint_polyline (joint_id, polyline_string, v1_on_side, v2_on_side)

for handler in logging.getLogger().handlers:
    if isinstance(handler,logging.FileHandler):  handler.close()