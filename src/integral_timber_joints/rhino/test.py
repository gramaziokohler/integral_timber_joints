# import profile
# from integral_timber_joints.geometry.joint import Joint
# from integral_timber_joints.geometry.beam import Beam
# from integral_timber_joints.geometry import JointHalfLap
# from compas.geometry import Frame, Point, Vector, Line
# from compas.geometry import intersection_segment_segment, distance_point_point


# def llx_distance(line1, line2):
#     dist_tol = line1.length * 1e-5
#     intersection_result = intersection_segment_segment(line1, line2, dist_tol)
#     if intersection_result[0] is None:
#         return None
#     if distance_point_point(intersection_result[0], intersection_result[1]) > dist_tol:
#         return None
#     return distance_point_point(intersection_result[0], line1.start)


# def test1():
#     beam1 = Beam(Frame(Point(-200, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0)))
#     beam2 = Beam(Frame(Point(-200, 500, 0), Vector(1, 0, 0), Vector(0, 1, 0)))
#     beam3 = Beam(Frame(Point(-200, 500, 100), Vector(1, 0, 0), Vector(0, 1, 0)))
#     beam4 = Beam(Frame(Point(0, -100, 100), Vector(0, 1, 0), Vector(-1, 0, 0)))
#     result = JointHalfLap.from_beam_beam_intersection(beam1, beam2)
#     result = JointHalfLap.from_beam_beam_intersection(beam1, beam3)
#     result = JointHalfLap.from_beam_beam_intersection(beam2, beam3)
#     result = JointHalfLap.from_beam_beam_intersection(beam2, beam4)
#     result = JointHalfLap.from_beam_beam_intersection(beam2, beam3)
#     result = JointHalfLap.from_beam_beam_intersection(beam3, beam4)
#     result = JointHalfLap.from_beam_beam_intersection(beam1, beam4)


# def test2():
#     line1 = Line([-100, 0, 0], [100, 0, 0])
#     line2 = Line([0, -100, 0], [0, 100, 0])
#     line3 = Line([0, -100, 10], [0, 100, 10])
#     llx_distance(line1, line2)
#     llx_distance(line1, line3)
#     llx_distance(line2, line3)

# def test3():
#     line1 = Line([-100, 0, 0], [100, 0, 0])
#     line2 = Line([0, -100, 0], [0, 100, 0])
#     line3 = Line([0, -100, 10], [0, 100, 10])

# def test4():
#     line1 = Line([-100, 0, 0], [100, 0, 0])
#     line2 = Line([0, -100, 0], [0, 100, 0])
#     line3 = Line([0, -100, 10], [0, 100, 10])
#     intersection_segment_segment(line1, line2, 1e-4)
#     intersection_segment_segment(line1, line3, 1e-4)
#     intersection_segment_segment(line2, line3, 1e-4)

# def test5():
#     intersection_segment_segment(([-100, 0, 0], [100, 0, 0]), ([0, -100, 0], [0, 100, 0]), 1e-4)
#     intersection_segment_segment(([-100, 0, 0], [100, 0, 0]), ([0, -100, 10], [0, 100, 10]), 1e-4)
#     intersection_segment_segment(([0, -100, 0], [0, 100, 0]), ([0, -100, 10], [0, 100, 10]), 1e-4)


# def test6():
#     beam1 = Beam(Frame(Point(-200, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0)))
#     beam1.reference_edge_wcf(1)
#     beam1.reference_edge_wcf(2)
#     beam1.reference_edge_wcf(3)

# if __name__ == "__main__":
#     import datetime

#     first_time = datetime.datetime.now()
#     for i in range(100):
#         test3()
#     later_time = datetime.datetime.now()
#     difference = later_time - first_time
#     print("Time took: %s" % difference)

#     first_time = datetime.datetime.now()
#     for i in range(100):
#         test4()
#     later_time = datetime.datetime.now()
#     difference = later_time - first_time
#     print("Time took: %s" % difference)

#     first_time = datetime.datetime.now()
#     for i in range(100):
#         test6()
#     later_time = datetime.datetime.now()
#     difference = later_time - first_time
#     print("Time took: %s" % difference)


from compas.geometry import Line
def test3():
    line1 = Line([-100, 0, 0], [100, 0, 0])
    line2 = Line([0, -100, 0], [0, 100, 0])
    line3 = Line([0, -100, 10], [0, 100, 10])
if __name__ == "__main__":
    import datetime
    first_time = datetime.datetime.now()
    for i in range(100):
        line1 = Line([-100, 0, 0], [100, 0, 0])
    later_time = datetime.datetime.now()
    difference = later_time - first_time
    print("Time took: %s" % difference)