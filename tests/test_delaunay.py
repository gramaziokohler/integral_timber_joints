from compas.geometry import conforming_delaunay_triangulation
points = [
    [0.0, 0.0, 0],
    [57.538885063467205, 0.0, 0],
    [145.87186036985872, -54.075563860662214, 0],
    [175.04623244383015, -64.890676632794793, 0],
    [98.061006836519709, 70.298233018861083, 0],
    [-106.15959768128255, 146.00402242378823, 0],
    [-29.174372073971767, 10.815112772132579, 0],
    [143.37573698827686, -9.276112734479657, 0],
    [13.859121611735034, 101.51237130513627, 0],
    [101.70537271666296, -27.037781930331107, 0],
    [159.2109847160535, -37.083394683637223, 0]
]
points = [[round(x) for x in point] for point in points]
points = [[p[0], p[1], 0] for p in points]
print(len(points))
v, f = conforming_delaunay_triangulation(points)
print ("Original len(v) = %i, Resulting len(v) = %i, len(f) = %s" % (len(points), len(v), len(f)))

