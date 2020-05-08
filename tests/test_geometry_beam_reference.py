from integral_timber_joints.geometry.beam import Beam
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector
import pytest

def test_beam_corners():
    beam = Beam(Frame(Point(0, 0, 0), Vector(0, 1, 0), Vector(0, 0, 1)),1000,100,150)
    assert beam.corner_ocf(1) == Point(0.000, 0.000, 0.000)
    assert beam.corner_ocf(2) == Point(0.000, 150.000, 0.000)
    assert beam.corner_ocf(3) == Point(0.000, 150.000, 100.000)
    assert beam.corner_ocf(4) == Point(0.000, 0.000, 100.000)
    assert beam.corner_ocf(5) == Point(1000.000, 0.000, 0.000)
    assert beam.corner_ocf(6) == Point(1000.000, 150.000, 0.000)
    assert beam.corner_ocf(7) == Point(1000.000, 150.000, 100.000)
    assert beam.corner_ocf(8) == Point(1000.000, 0.000, 100.000)

    with pytest.raises(IndexError):
        c = beam.corner_ocf(0)
    with pytest.raises(IndexError):
        c = beam.corner_ocf(9)

def test_beam_reference_side():
    beam = Beam(Frame(Point(0, 0, 0), Vector(0, 1, 0), Vector(0, 0, 1)),1000,100,150)
    assert beam.refernce_side_ocf(1) == Frame(Point(0.000, 0.000, 0.000), Vector(1.000, 0.000, 0.000), Vector(-0.000, 0.000, 1.000))
    assert beam.refernce_side_ocf(2) == Frame(Point(0.000, 150.000, 0.000), Vector(1.000, 0.000, 0.000), Vector(0.000, -1.000, 0.000))
    assert beam.refernce_side_ocf(3) == Frame(Point(0.000, 150.000, 100.000), Vector(1.000, 0.000, 0.000), Vector(0.000, 0.000, -1.000))
    assert beam.refernce_side_ocf(4) == Frame(Point(0.000, 0.000, 100.000), Vector(1.000, 0.000, 0.000), Vector(0.000, 1.000, 0.000))
    assert beam.refernce_side_ocf(5) == Frame(Point(0.000, 0.000, 0.000), Vector(0.000, 0.000, 1.000), Vector(0.000, 1.000, -0.000))
    assert beam.refernce_side_ocf(6) == Frame(Point(1000.000, 150.000, 0.000), Vector(0.000, 0.000, 1.000), Vector(0.000, -1.000, 0.000))
    with pytest.raises(IndexError):
        c = beam.refernce_side_ocf(7)
    with pytest.raises(IndexError):
        c = beam.refernce_side_ocf(0)

def test_beam_corners_not_affected_by_wcf():
    beam1 = Beam(Frame(Point(0, 0, 0), Vector(0, 1, 0), Vector(0, 0, 1)),1000,100,150)
    beam2 = Beam(Frame(Point(10, 20, 30), Vector(0, -1, 0), Vector(-1, 0, 0)),1000,100,150)
    for i in range (1, 9):
        assert beam1.corner_ocf(i) == beam2.corner_ocf(i)


if __name__ == "__main__":
    test_beam_corners()
    test_beam_reference_side()
    test_beam_corners_not_affected_by_wcf()
