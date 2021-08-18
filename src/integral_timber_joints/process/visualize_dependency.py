from integral_timber_joints.process import ComputationalDependency
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import random
from itertools import combinations
from math import copysign
import math

RADIUS = 100
PAUSE = 0.1


def initialize_graph(dependency: ComputationalDependency):
    for node in dependency.nodes(False):
        dependency.node_attribute(node, 'x', random.uniform(-1, 1))
        dependency.node_attribute(node, 'y', random.uniform(-1, 1))
        dependency.node_attribute(node, 'colour', random.uniform(0, 1))


def draw_graph(dependency: ComputationalDependency):
    plt.figure(figsize=(8, 8))
    plt.axis('equal')

    # Drawing lines - edges
    for edge in dependency.edges():
        x = [dependency.node_attribute(node, 'x') for node in edge]
        y = [dependency.node_attribute(node, 'y') for node in edge]
        plt.plot(x, y, linestyle='solid', color='grey', zorder=1)

    # Drawing edges
    x = []
    y = []
    area = []
    colours = []
    for node in dependency.nodes(False):
        _x, _y = dependency.node_attributes(node, 'xy')
        x.append(_x)
        y.append(_y)
        colours.append(dependency.node_attribute(node, 'colour'))
        area.append(RADIUS)
        plt.annotate(node, (_x, _y), ha='center')
    plt.scatter(x, y, s=area, c=colours, alpha=0.5, zorder=2)

    plt.show()


def total_edge_lengths(dependency: ComputationalDependency):
    lengths = 0
    for edge in dependency.edges():
        lengths += edge_length(dependency, edge)
    return lengths


def edge_length(dependency: ComputationalDependency, edge):
    a, b = edge
    a_x, a_y = dependency.node_attributes(a, 'xy')
    b_x, b_y = dependency.node_attributes(b, 'xy')
    length = math.sqrt(math.pow(a_x - b_x, 2) + math.pow(a_y - b_y, 2))
    return length


def edge_has_crossing(dependency: ComputationalDependency, edge1, edge2):
    # Ignore shared edge
    if edge1[0] in edge2:
        return False
    if edge1[1] in edge2:
        return False

    def line(edge):
        pt1, pt2 = edge
        x1, y1 = dependency.node_attributes(pt1, 'xy')
        x2, y2 = dependency.node_attributes(pt2, 'xy')
        return (x1, y1, x2, y2)
    line1 = line(edge1)
    line2 = line(edge2)

    def points_on_opposite(line1, line2):
        x1, y1, x2, y2 = line1
        x = line2[0]
        y = line2[1]
        test1 = (x - x1)*(y2-y1) - (y-y1)*(x2-x1)
        x = line2[2]
        y = line2[3]
        test2 = (x - x1)*(y2-y1) - (y-y1)*(x2-x1)
        return test1 * test2 < 0

    return points_on_opposite(line1, line2) and points_on_opposite(line2, line1)


def total_crossed_edges(dependency: ComputationalDependency):
    count = 0
    for edge1, edge2 in combinations(list(dependency.edges()), 2):
        if edge_has_crossing(dependency, edge1, edge2):
            count += 1
    return count


def optimize_position(dependency: ComputationalDependency):

    # for node in dependency.nodes(False):
    #     dependency.node_attribute(node, 'x', random.uniform(-1, 1))
    #     dependency.node_attribute(node, 'y', random.uniform(-1, 1))

    # Pull Neighbours behaviour
    PULL_THRESHOLD = 0.15
    PULL_RATIO = 0.9
    for a, b in dependency.edges():
        current_length = edge_length(dependency, (a, b))
        if current_length < PULL_THRESHOLD:
            continue
        move_amount = (current_length - PULL_THRESHOLD) * PULL_RATIO / 2
        move_ratio = move_amount / current_length

        a_x, a_y = dependency.node_attributes(a, 'xy')
        b_x, b_y = dependency.node_attributes(b, 'xy')

        diff_x = (a_x - b_x) * move_ratio
        diff_y = (a_y - b_y) * move_ratio
        a_x = a_x - diff_x
        b_x = b_x + diff_x
        a_y = a_y - diff_y
        b_y = b_y + diff_y

        dependency.node_attributes(a, 'xy', [a_x, a_y])
        dependency.node_attributes(b, 'xy', [b_x, b_y])

    # Push away behaviour
    def push(PUSH_THRESHOLD, PUSH_RATIO):
        for edge in combinations(list(dependency.nodes()), 2):
            a, b = edge
            if edge_length(dependency, edge) > PUSH_THRESHOLD:
                continue
            a_x, a_y = dependency.node_attributes(a, 'xy')
            b_x, b_y = dependency.node_attributes(b, 'xy')
            if abs(a_x - b_x) < PUSH_THRESHOLD:
                diff = (PUSH_THRESHOLD - abs(a_x - b_x)) * PUSH_RATIO / 2
                diff = copysign(diff, (a_x - b_x))
                a_x = a_x + diff
                b_x = b_x - diff
            if abs(a_y - b_y) < PUSH_THRESHOLD:
                diff = (PUSH_THRESHOLD - abs(a_y - b_y)) * PUSH_RATIO / 2
                diff = copysign(diff, (a_y - b_y))
                a_y = a_y + diff
                b_y = b_y - diff
            dependency.node_attributes(a, 'xy', [a_x, a_y])
            dependency.node_attributes(b, 'xy', [b_x, b_y])

    def push_y(PUSH_THRESHOLD, PUSH_RATIO):
        for edge in combinations(list(dependency.nodes()), 2):
            a, b = edge
            a_x, a_y = dependency.node_attributes(a, 'xy')
            b_x, b_y = dependency.node_attributes(b, 'xy')
            if abs(a_y - b_y) < PUSH_THRESHOLD:
                diff = (PUSH_THRESHOLD - abs(a_y - b_y)) * PUSH_RATIO / 2
                diff = copysign(diff, (a_y - b_y))
                a_y = a_y + diff
                b_y = b_y - diff
            dependency.node_attribute(a, 'y', a_y)
            dependency.node_attribute(b, 'y', b_y)

    push(0.25, 1)  # Avoid overlap
    push(0.8, 0.2)  # Fill empty space
    push_y(0.1, 1)  # Avoid text overlap

    # Begining functions Baloon upwards
    UP_FORCE = 0.1
    for node in dependency.get_beginning_function_names():
        dependency.node_attribute(node, 'y', dependency.node_attribute(node, 'y') + UP_FORCE)

    # Set top and bottom
    TOP_LIMIT = 0.8
    BOTTOM_LIMIT = -0.9
    LEFT_LIMIT = -0.8
    RIGHT_LIMIT = 0.8
    for node in dependency.nodes():
        if node in dependency.get_beginning_function_names():
            if dependency.node_attribute(node, 'y') < TOP_LIMIT:
                dependency.node_attribute(node, 'y', TOP_LIMIT)
        elif node in dependency.get_terminal_function_names():
            if dependency.node_attribute(node, 'y') > BOTTOM_LIMIT:
                dependency.node_attribute(node, 'y', BOTTOM_LIMIT)
        else:
            # Those in between
            if dependency.node_attribute(node, 'y') > TOP_LIMIT-0.1:
                dependency.node_attribute(node, 'y', TOP_LIMIT-0.1)
            if dependency.node_attribute(node, 'y') < BOTTOM_LIMIT+0.1:
                dependency.node_attribute(node, 'y', BOTTOM_LIMIT+0.1)

    for node in dependency.nodes():
        if dependency.node_attribute(node, 'x') > RIGHT_LIMIT:
            dependency.node_attribute(node, 'x', RIGHT_LIMIT)
        if dependency.node_attribute(node, 'x') < LEFT_LIMIT:
            dependency.node_attribute(node, 'x', LEFT_LIMIT)


def optimize_one_graph(iterations):
    dependency = ComputationalDependency()
    initialize_graph(dependency)
    for i in range(iterations):
        optimize_position(dependency)
    length = total_edge_lengths(dependency)
    crossed_edges = total_crossed_edges(dependency)
    return(length, crossed_edges, dependency)


if __name__ == '__main__':

    results = []
    for i in range(50):
        random.seed(i)
        result = optimize_one_graph(100)
        results.append(result)
        # print("total_edge_lengths:", result[0])
        print("total_crossed_edges:", result[1])
        # if result[1] == 0:
        #     break
    results.sort(key=lambda x: x[1])
    best_crossing_count = results[0][1]
    print("Crossed Edges in best Iteration: ", best_crossing_count)
    results = [result for result in results if result[1] == best_crossing_count]
    print("Number of results similar: ", len(results))
    results.sort(key=lambda x: x[0])
    chosen_graph = results[0][2]
    print("Showing the Shortest graph edges:")
    draw_graph(chosen_graph)
    results.sort(key=lambda x: x[0], reverse=True)
    chosen_graph = results[0][2]
    draw_graph(chosen_graph)
