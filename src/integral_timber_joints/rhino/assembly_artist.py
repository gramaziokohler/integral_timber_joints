import uuid
import Rhino.Geometry as rg
import rhinoscriptsyntax as rs
import scriptcontext as sc  # type: ignore
from compas.datastructures import Mesh
from compas.geometry import Cylinder, Polyhedron
from compas_rhino.utilities import clear_layer, delete_objects, draw_breps, draw_cylinders, draw_mesh
from Rhino.DocObjects.ObjectColorSource import ColorFromObject  # type: ignore
from System.Drawing import Color  # type: ignore

from integral_timber_joints.assembly import Assembly
from integral_timber_joints.rhino.artist import mesh_to_brep, vertices_and_faces_to_brep_struct
from integral_timber_joints.rhino.utility import purge_objects

add_brep = sc.doc.Objects.AddBrep

TOL = sc.doc.ModelAbsoluteTolerance

guid = uuid.UUID

class AssemblyNurbsArtist(object):
    """ Artist to draw Beams of an Assembly in Rhino with Nurbs geometry
    Items are drawn in specific layers for quickly turning them on and off.
    Beam Brep : layer = itj::beams_brep         name = beam_id

    Dictionary keep track of the objects drawn in Rhino:
    self.beam_guids

    Beams are drawn in their final location.

    """

    def __init__(self, assembly):
        # type: (Assembly) -> None
        """Create new AssemblyNurbsArtist for an Assembly in Rhino Context"""

        self.assembly = assembly  # type: Assembly

        # # Create guid in dictionary to store geometries added to Rhino document
        self._beam_guids = {}  # type: dict[str, list[str]]

        self.beam_layer_name = 'itj::beams_brep::final'

        # Empty existing layers and create them if not existing.
        # This also creates the guid dictionary
        if self.assembly is not None:
            self.empty_layers()

    @property
    def all_layer_names(self):
        yield self.beam_layer_name

    def beam_guids(self, beam_id):
        # type: (str) -> list[guid]
        """Returns the guids of a beam.
        Creates dictionary entry if it is not yet tracked.
        """
        if beam_id not in self._beam_guids:
            self._beam_guids[beam_id] = []
        return self._beam_guids[beam_id]

    def empty_layers(self):
        # type:() -> None
        """Clear the default artist layers in Rhino.
        Create those layers if doesnt exist
        Gguid dictionary is reset to all empty lists.
        """
        # Clear layer if exist otherwise, create new layer
        for layer_name in self.all_layer_names:
            if not rs.IsLayer(layer_name):
                rs.AddLayer(layer_name)
            else:
                clear_layer(layer_name)

        # Create new guid dictionary / clear out previously-saved guids.
        for beam_id in self.assembly.beam_ids():
            self.delete_beam(beam_id, redraw=False)

    def delete_beam(self, beam_id, redraw=True):
        # type:(str, bool) -> None
        """ Delete visualization geometry geometry (brep, mesh, tag etc) related to a beam.
        Tools are not affected.
        Stored guid reference is also removed.

        If beam_id is not yet tracked in self.guid, the new entry will be created.
        """
        rs.EnableRedraw(False)
        if beam_id in self.assembly.beam_ids():
            if len(self.beam_guids(beam_id)) > 0:
                purge_objects(self.beam_guids(beam_id), redraw=False)
                del self.beam_guids(beam_id)[:]
        if redraw:
            rs.EnableRedraw(True)

    def draw_beam(self, beam_id, delete_old=False, redraw=True, verbose=False):
        # type: (str, bool, bool, bool) -> None
        """Function to draw specified beam with nurbs.

        By default `delete_old` is False.

        `redraw` triggers the Rhino canvas to refresh.
        """
        rs.EnableRedraw(False)

        # If not delete_old, and there are already items drawn, we preserve them.
        if len(self.beam_guids(beam_id)) > 0 and not delete_old:
            pass
        else:
            # Layer
            rs.CurrentLayer(self.beam_layer_name)

            # Positive geometry is the uncut beam mesh
            mesh_box = self.assembly.beam(beam_id).draw_uncut_mesh()
            positive_brep_guids = draw_breps(mesh_to_brep(mesh_box), join=True, redraw=False)

            # Collect all the negative features
            joints = self.assembly.get_joints_of_beam(beam_id)
            beam_cuts = self.assembly.beam_cuts(beam_id)
            features = joints
            features += beam_cuts

            if len(features) > 0:
                guids = []  # Hold the guids of the final boolean result
                negative_brep_guids = []
                # Get the negative meshes from the features and convert them to nurbs
                for feature in features:
                    for negative_shape in feature.get_feature_shapes(self.assembly.beam(beam_id)):
                        print(negative_shape.__class__)
                        if isinstance(negative_shape, Polyhedron):
                            vertices_and_faces = negative_shape.to_vertices_and_faces()
                            struct = vertices_and_faces_to_brep_struct(vertices_and_faces)
                            print("Polyhedron :", struct)
                            guids = draw_breps(struct, join=True, redraw=False)
                            negative_brep_guids.extend(guids)
                        elif isinstance(negative_shape, Cylinder):
                            cylinder = negative_shape
                            start = cylinder.center + cylinder.normal.scaled(cylinder.height / 2)
                            end = cylinder.center - cylinder.normal.scaled(cylinder.height / 2)
                            struct = {'start': list(start), 'end': list(end), 'radius': cylinder.circle.radius}
                            print("Cylinder : ", struct)
                            guids = draw_cylinders([struct], cap=True, redraw=False)
                            negative_brep_guids.extend(guids)

                # Perform Boolean Difference
                positive_breps = [rs.coercebrep(guid) for guid in positive_brep_guids]
                negative_breps = [rs.coercebrep(guid) for guid in negative_brep_guids]
                print(positive_breps, negative_breps)
                boolean_result = rg.Brep.CreateBooleanDifference(positive_breps, negative_breps, TOL)
                if boolean_result is None:
                    print("Artist draw_beam(%s) Boolean Failure"% beam_id)
                else:
                    for brep in boolean_result:
                        guid = add_brep(brep)
                        if guid:
                            guids.append(guid)

                    # Save resulting guid(s) into dictionary
                    self.beam_guids(beam_id).extend(guids)

                    # Delete the original boolean set geometries
                    delete_objects(positive_brep_guids + negative_brep_guids, purge=True, redraw=False)
            else:
                self.beam_guids(beam_id).extend(positive_brep_guids)

        if redraw:
            rs.EnableRedraw(True)
