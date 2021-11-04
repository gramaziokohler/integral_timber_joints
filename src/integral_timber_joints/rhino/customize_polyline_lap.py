import rhinoscriptsyntax as rs
from compas.geometry import Frame, Point, Vector, Transformation, distance_point_point
from compas_rhino.geometry import RhinoPoint

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
print (model_frame)
T = Transformation.from_frame(model_frame)

# Decode Input String
input_string = "[[-3.3315692002128099, 1.03251625305613, 2.4500000000000002], [-0.98480800000000002, 0.173648, 0.0], [0.33682400000000001, -0.059390999999999999, 0.939693], [0.98480800000000002, -0.173648, 0.0], [-0.33682400000000001, 0.059390999999999999, -0.939693], '[50.,[[[0.,0.],[0.1,0.],[0.11,0.1],[0.5,0.2],[1.,0.]],[[0.,0.],[0.5,0.15],[0.9,0.11],[1.,0.1],[1.,0.]],[[0.,0.],[0.1,0.],[0.11,0.1],[0.5,0.2],[1.,0.]],[[0.,0.],[0.5,0.15],[0.9,0.11],[1.,0.1],[1.,0.]]]]']"
pt, v1, v2, v3, v4, polyline_string = eval(input_string)
pt = Point(*[p * 1000 for p in pt]).transformed(T)
v1 = Vector(*v1).transformed(T).unitized()
v2 = Vector(*v2).transformed(T).unitized()
v3 = Vector(*v3).transformed(T).unitized()
v4 = Vector(*v4).transformed(T).unitized()

print (pt, v1, v2, v3, v4)

from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointPolylineLap
process = get_process()
artist = get_process_artist()
assembly = process.assembly

def find_joint_by_centroid(centroid_pt, distance_threshold = 50):

    for joint_id in assembly.joint_ids():
        joint = assembly.joint(joint_id)
        if joint.__class__.__name__ != "JointPolylineLap":
            continue
        distance = distance_point_point(joint.centroid, centroid_pt)
        if distance < distance_threshold:
            print ("Joint Found: %s, %s" % (joint_id, distance))
            return joint_id

joint_id = find_joint_by_centroid(pt)
joint = assembly.joint(joint_id) # type: JointPolylineLap

# * Find out the vector alignment
u1 = Vector.from_start_end(joint.corner_pts[3], joint.corner_pts[0]).unitized()
u2 = Vector.from_start_end(joint.corner_pts[0], joint.corner_pts[1]).unitized()

dot_products = [u1.dot(v1), u1.dot(v2), u1.dot(v3), u1.dot(v4)]
u1_alignemnt = dot_products.index(max(dot_products))
dot_products = [u2.dot(v1), u2.dot(v2), u2.dot(v3), u2.dot(v4)]
u2_alignemnt = dot_products.index(max(dot_products))
print (u1_alignemnt)
print (u2_alignemnt)

# Set polyline
nbr_joint_id = (joint_id[1], joint_id[0])
assembly.joint(joint_id).param_string = polyline_string
assembly.joint(nbr_joint_id).param_string = polyline_string

# TODO Flipping and turning

# Update the interactive beam
artist.redraw_interactive_beam(joint_id[0], redraw=False)
artist.redraw_interactive_beam(joint_id[1], redraw=True)