
import Rhino 
import rhinoscriptsyntax as rs
import scriptcontext

def SelectBeamFace(prompt):
    filter = Rhino.DocObjects.ObjectType.Surface
    #rc, objref = Rhino.Input.RhinoGet.GetOneObject("Select First Beam Surface", False, filter)
    go = Rhino.Input.Custom.GetObject() 
    go.GeometryFilter = Rhino.DocObjects.ObjectType.Surface;
    go.DisablePreSelect()
    go.SetCommandPrompt(prompt)
    result = go.Get()
    guid = go.Object(0).ObjectId
    face = go.Object(0).Face()
    return guid, face
    
    
def GetJointDepth():
    jointDepth = 50
    # restore stickyval if it has been saved
    if scriptcontext.sticky.has_key("my_joint_depth"):
        jointDepth = scriptcontext.sticky["my_joint_depth"]

    gn = Rhino.Input.Custom.GetNumber()
    gn.SetDefaultNumber(jointDepth)
    gn.SetCommandPrompt("Half Lap Joint Depth")
    gn.SetLowerLimit(1.0, False)
    gn.Get()
    if (gn.CommandResult() == Rhino.Commands.Result.Success):
        scriptcontext.sticky["my_joint_depth"] = gn.Number()
        return gn.Number()
    else:
        return jointDepth
    
    
    
def RunCommand(is_interactive):
    beam1Guid, beam1Face = SelectBeamFace("Select First Beam Face") # BrepFace 
    beam1Brep = beam1Face.Brep

    beam2Guid, beam2Face = SelectBeamFace("Select Second Beam Face")
    beam2Brep = beam2Face.Brep

    #Compute Intersection

    loop1 = beam1Face.OuterLoop.To3dCurve()
    loop2 = beam2Face.OuterLoop.To3dCurve()
    
    
    # Calculate the intersection
    result = Rhino.Geometry.Curve.CreateBooleanIntersection(loop1,loop2)
    intersectionCurve = result[0]
    # Create the boolean solid
    jointDepth = GetJointDepth()
    intersectionSolid = Rhino.Geometry.Extrusion.Create(intersectionCurve,-jointDepth,True)
    
    #Draw the boolean Box
    #scriptcontext.doc.Objects.AddExtrusion(intersectionSolid)
    
    tolerance = scriptcontext.doc.ModelAbsoluteTolerance;
    #Boolean for the broken piece
    result = Rhino.Geometry.Brep.CreateBooleanDifference(beam2Brep, intersectionSolid.ToBrep(), tolerance)
    if not result: return Rhino.Commands.Result.Nothing
    jointedBeam2 = result[0]
    #Boolean for the broken piece
    result = Rhino.Geometry.Brep.CreateBooleanDifference(beam1Brep, jointedBeam2, tolerance)
    if not result: return Rhino.Commands.Result.Nothing
    jointedBeam1 = result[0]

    #Add New Beams
    #scriptcontext.doc.Objects.AddBrep(jointedBeam1)
    #scriptcontext.doc.Objects.AddBrep(jointedBeam2)
    
    #Replace Old Beams
    scriptcontext.doc.Objects.Replace(beam1Guid,jointedBeam1)
    scriptcontext.doc.Objects.Replace(beam2Guid,jointedBeam2)
    #scriptcontext.doc.Objects.Delete(beam2Brep)



if __name__ == '__main__':
    RunCommand(True) 