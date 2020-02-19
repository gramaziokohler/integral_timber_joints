from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json

from compas_fab.robots import JointTrajectoryPoint

from compas.datastructures import Mesh
from compas.datastructures import mesh_transform
from compas.geometry import Frame

from .utilities import _deserialize_from_data
from .utilities import _serialize_to_data

__all__ = ['Element']


class Element(object):
    """Data structure representing a discrete elements of an assembly.

    Attributes
    ----------
    frame : :class:`compas.geometry.Frame`
        The frame of the element.

    Examples
    --------
    >>> from compas.datastructures import Mesh
    >>> from compas.geometry import Box
    >>> element = Element.from_box(Box(Frame.worldXY(), ))

    """

    def __init__(self, frame):
        super(Element, self).__init__()
        self.frame = frame
        self.trajectory = None
        self._gripping_frame = None
        self._source = None
        self._mesh = None

    @classmethod
    def from_mesh(cls, mesh, frame):
        """Construct an element from a mesh.

        Parameters
        ----------
        mesh : :class:`Mesh`
            Mesh datastructure.
        frame : :class:`Frame`
            Origin frame of the element.

        Returns
        -------
        :class:`Element`
            New instance of element.
        """
        element = cls(frame)
        element._source = mesh
        return element

    @classmethod
    def from_shape(cls, shape, frame):
        """Construct an element from a shape primitive.

        Parameters
        ----------
        shape : :class:`compas.geometry.Shape`
            Shape primitive describing the element.
        frame : :class:`Frame`
            Origin frame of the element.

        Returns
        -------
        :class:`Element`
            New instance of element.
        """
        element = cls(frame)
        element._source = shape
        return element

    @classmethod
    def from_box(cls, box):
        """Construct an element from a box primitive.

        Parameters
        ----------
        box : :class:`compas.geometry.Box`
            Box primitive describing the element.

        Returns
        -------
        :class:`Element`
            New instance of element.
        """
        return cls.from_shape(box, box.frame)

    @property
    def mesh(self):
        """Mesh of the element."""
        if not self._source:
            return None

        if self._mesh:
            return self._mesh

        if isinstance(self._source, Mesh):
            self._mesh = self._source
        else:
            self._mesh = Mesh.from_shape(self._source)

        return self._mesh

    @mesh.setter
    def mesh(self, mesh):
        self._source = self._mesh = mesh

    @property
    def frame(self):
        """Frame of the element."""
        return self._frame

    @frame.setter
    def frame(self, frame):
        self._frame = frame.copy()

    @property
    def gripping_frame(self):
        """Gripping frame of the element."""
        if not self._gripping_frame:
            self._gripping_frame = self.frame.copy()

        return self._gripping_frame

    @gripping_frame.setter
    def gripping_frame(self, frame):
        self._gripping_frame = frame.copy() if frame else None

    @property
    def centroid(self):
        return self.mesh.centroid()

    @classmethod
    def from_data(cls, data):
        """Construct an element from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        Element
            The constructed element.
        """
        element = cls(Frame.worldXY())
        element.data = data
        return element

    @property
    def data(self):
        """Returns the data dictionary that represents the element.

        Returns
        -------
        dict
            The element data.

        Examples
        --------
        >>> element = Element(Frame.worldXY())
        >>> print(element.data)
        """
        d = dict(frame=self.frame.to_data())

        # Only include gripping plane if attribute is really set
        # (unlike the property getter that defaults to `self.frame`)
        if self._gripping_frame:
            d['gripping_frame'] = self.gripping_frame.to_data()

        if self._source:
            d['_source'] = _serialize_to_data(self._source)

        # Probably best to store JointTrajectory instead of JointTrajectoryPoints
        if self.trajectory:
            d['trajectory'] = [p.to_data() for p in self.trajectory]
            
        return d

    @data.setter
    def data(self, data):
        self.frame = Frame.from_data(data['frame'])
        if 'gripping_frame' in data:
            self.gripping_frame = Frame.from_data(data['gripping_frame'])
        if '_source' in data:
            self._source = _deserialize_from_data(data['_source'])
        if 'trajectory' in data:
            self.trajectory = [JointTrajectoryPoint.from_data(d) for d in data['trajectory']]

    def to_data(self):
        """Returns the data dictionary that represents the element.

        Returns
        -------
        dict
            The element data.

        Examples
        --------
        >>> from compas.geometry import Frame
        >>> e1 = Element(Frame.worldXY())
        >>> e2 = Element.from_data(element.to_data())
        >>> e2.frame == Frame.worldXY()
        True
        """
        return self.data

    def transform(self, transformation):
        """Transforms the element.

        Parameters
        ----------
        transformation : :class:`Transformation`

        Returns
        -------
        None

        Examples
        --------
        >>> from compas.geometry import Box
        >>> from compas.geometry import Translation
        >>> element = Element.from_box(Box(Frame.worldXY(), 1, 1, 1))
        >>> element.transform(Translation([1, 0, 0]))
        """
        self.frame.transform(transformation)
        if self._gripping_frame:
            self.gripping_frame.transform(transformation)
        if self._source:
            if type(self._source) == Mesh:
                mesh_transform(self._source, transformation)  # it would be really good to have Mesh.transform()
            else:
                self._source.transform(transformation)
    
    def transformed(self, transformation):
        """Returns a transformed copy of this element.

        Parameters
        ----------
        transformation : :class:`Transformation`

        Returns
        -------
        Element

        Examples
        --------
        >>> from compas.geometry import Box
        >>> from compas.geometry import Translation
        >>> element = Element.from_box(Box(Frame.worldXY(), 1, 1, 1))
        >>> element2 = element.transformed(Translation([1, 0, 0]))
        """
        elem = self.copy()
        elem.transform(transformation)
        return elem

    def copy(self):
        """Returns a copy of this element.

        Returns
        -------
        Element
        """
        elem = Element(self.frame.copy())
        if self._gripping_frame:
            elem.gripping_frame = self.gripping_frame.copy()
        if self._source:
            elem._source = self._source.copy()
        return elem
