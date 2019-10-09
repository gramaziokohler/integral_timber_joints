import rhinoscriptsyntax as rs

import Rhino
import compas


import integral_timber_joints


def RunCommand( is_interactive ):

    #load Derivation and model
    derivation = Derivation.from_json(rhino_UI_utilities.get_json_file_location())
    model = derivation.get_next_step()

    #user input
    rc, corners = Rhino.Input.RhinoGet.GetRectangle()
    if rc != Rhino.Commands.Result.Success:
        return rc
    plane = Rhino.Geometry.Plane(corners[0], corners[1], corners[2])
    beam_frame = Frame(plane[0],plane[1],plane[2])
    length = rs.GetReal("length",4000,300,None)

    #Generate unique name for the Beam 
    name = create_id()
    
    #Create Beam 
    model.rule_create_beam(beam_frame,length,100,100,name)

    
    #Save Derivation (Model is also saved)
    derivation.to_json(rhino_UI_utilities.get_json_file_location(), pretty = True)
    
    #Visualization
    artist = MeshArtist(None, layer ='BEAM::Beams_out')
    artist.clear_layer()
    for beam in model.beams:
        artist = MeshArtist(beam.mesh, layer ='BEAM::Beams_out')#.mesh is not ideal fix in beam and assemble class
        artist.draw_faces(join_faces=True)
    return 0

if __name__ == '__main__':
    pass
    #RunCommand(True)
