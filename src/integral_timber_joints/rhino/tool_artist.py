import compas_rhino
import scriptcontext as sc
from compas_rhino.artists import RobotModelArtist
from Rhino.DocObjects.ObjectColorSource import ColorFromLayer, ColorFromObject
from Rhino.DocObjects.ObjectMaterialSource import MaterialFromObject
from System.Drawing import Color


class ToolArtist(RobotModelArtist):
    def _exit_layer(self):
        if self.layer and self._previous_layer:
            compas_rhino.rs.CurrentLayer(self._previous_layer)
    
    # Hot fix to add guid return function to compas artist
    # Overridding compas_rhino RobotModelArtist's drawvisual to return stuff
    def draw_visual(self):
        """Draws all visual geometry of the robot model. Return list of newly created Rhino object's GUID."""
        visuals = super(RobotModelArtist, self).draw_visual()
        visuals = list(visuals)

        self._enter_layer()

        new_guids = []
        for mesh in visuals:
            guids = self._add_mesh_to_doc(mesh)
            new_guids.extend(guids)

        self._exit_layer()
        return new_guids

    def _add_mesh_to_doc(self, mesh):
        guid = sc.doc.Objects.AddMesh(mesh)

        color = None
        if 'MeshColor.R' in mesh.UserDictionary:
            color = [mesh.UserDictionary['MeshColor.R'],
                     mesh.UserDictionary['MeshColor.G'],
                     mesh.UserDictionary['MeshColor.B'],
                     mesh.UserDictionary['MeshColor.A']]
        name = mesh.UserDictionary['MeshName'] if 'MeshName' in mesh.UserDictionary else None

        obj = sc.doc.Objects.Find(guid)

        if obj:
            attr = obj.Attributes
            if color:
                r, g, b, a = [i * 255 for i in color]
                attr.ObjectColor = Color.FromArgb(a, r, g, b)
                attr.ColorSource = ColorFromObject

                material_name = 'robotmodelartist.{:.2f}_{:.2f}_{:.2f}_{:.2f}'.format(r, g, b, a)
                material_index = sc.doc.Materials.Find(material_name, True)

                # Material does not exist, create it
                if material_index == -1:
                    material_index = sc.doc.Materials.Add()
                    material = sc.doc.Materials[material_index]
                    material.Name = material_name
                    material.DiffuseColor = attr.ObjectColor
                    material.CommitChanges()

                attr.MaterialIndex = material_index
                attr.MaterialSource = MaterialFromObject
            else:
                attr.ColorSource = ColorFromLayer

            if name:
                attr.Name = name

            obj.CommitChanges()
        return [guid]
