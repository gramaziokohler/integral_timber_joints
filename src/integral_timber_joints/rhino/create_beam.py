import Rhino
import compas
from integral_timber_joints.datastructures.beam import Beam
from pprint import pprint

def create_beam_from_brep (rhino_brep, edge_min_length = 1.0e-6):
    assert type(rhino_brep) == Rhino.Geometry.Brep
    # Gather edges and create a unique set of edges (lines)
    edge_lengths = []
    for edge in rhino_brep.Edges:
        if not edge.IsLinear(): continue
        edge_length = edge.GetLength()
        if (edge_length < edge_min_length): continue
        # Check if the edge has a colinear friend in the list
        # compas.geometry.Line(compas.rhino.)
        edge_line = Rhino.Geometry.Line(edge.PointAtStart, edge.PointAtEnd)
        # If yes, add the length to the corrispond entry
        edge_found = False
        for recorded_line in edge_lengths:
            if is_colinear(recorded_line[0],edge_line)
        #     i
        edge_lengths.append([edge_line,edge_length])

    pprint(edge_lengths)
    #new_beam = Beam(beam_frame,length,width,height)
    new_beam = None
    return new_beam
    print("Done")


def linear_curve_to_vector(curve):
    
    # type: (Rhino.Geometry.Curve) -> Rhino.Geometry.Vector3d
    start_point = curve.PointAtStart
    end_point = curve.PointAtEnd
    vector = Rhino.Geometry.Vector3d(end_point - start_point)
    return vector



def create_beam_from_mesh (rhino_mesh):
    raise NotImplementedError

def create_beam_from_line (rhino_line, rhino_vector):
    raise NotImplementedError

def FindFirstObjectByName(name):
    import scriptcontext
    import System.Guid

    settings = Rhino.DocObjects.ObjectEnumeratorSettings()
    settings.NameFilter = name
    for rhobj in scriptcontext.doc.Objects.GetObjectList(settings):
        print("Found Object" + str(type(rhobj.Geometry)))
        return rhobj
    return None
    # if not ids:
    #     print "No objects with the name", name
    #     return None
    # else:
    #     print "Found", len(ids), "objects"
    #     for id in ids: print "  ", id
    # return Rhino.Commands.Result.Success


if __name__ == '__main__':
    
    #Select object named "brep_sample" for testing
    objref = FindFirstObjectByName("brep_sample")
    
    #If the object make sense, it is passed to function create_beam_from_brep()
    if type(objref) != Rhino.DocObjects.BrepObject:
        print("This is not a Brep")
    else:
        brep_geometry = objref.Geometry
        beam = create_beam_from_brep(brep_geometry)
        print(beam)