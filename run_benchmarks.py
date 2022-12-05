#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost

SOLVER = "CBS"
MAX_TIME = 300


def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_map(filename):
    file_path = 'maps/' + filename
    f = Path(file_path)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(file_path, 'r')
    # first line is not important
    f.readline()
    line = f.readline()
    _, rows = line.split()
    line = f.readline()
    _, columns = line.split()
    rows = int(rows)
    # columns = int(columns)
    # another unimportant line in the map file
    f.readline()
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    f.close()
    return my_map


def import_mapf_instance(filename, num_agents):
    f = Path(filename)
    if not f.is_file():
        print('hello')
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: is the version not important
    f.readline()
    line = f.readline()
    _, map_instance, rows, columns, start_x, start_y, goal_x, goal_y, _ = line.split()
    rows = int(rows)
    columns = int(columns)
    # load map
    my_map = import_map(map_instance)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    if not my_map[int(start_x)][int(start_y)] and not my_map[int(goal_x)][int(goal_y)]:
        starts.append((int(start_x), int(start_y)))
        goals.append((int(goal_x), int(goal_y)))
    line = f.readline()
    lines_read = 1
    while line and lines_read < num_agents:
        _, _, _, _, sx, sy, gx, gy, _ = line.split()
        if not my_map[int(sx)][int(sy)] and not my_map[int(gx)][int(gy)]:
            starts.append((int(sx), int(sy)))
            goals.append((int(gx), int(gy)))
            lines_read += 1
        line = f.readline()
    f.close()
    if not line or num_agents > 64:
        eof_reached = True
    else:
        eof_reached = False
    return eof_reached, my_map, starts, goals


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms on benchmark maps')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))
    parser.add_argument('--time', type=int, default=MAX_TIME,
                        help='Set max time for cbs solver')

    args = parser.parse_args()

    result_file = open("benchmarks.csv", "w", buffering=1)
    if args.solver == "All":
        folders = [
            'empty-8-8.map-scen-even/scen-even',
            'empty-8-8.map-scen-random/scen-random',
            'empty-16-16.map-scen-even/scen-even',
            'empty-16-16.map-scen-random/scen-random',
            'empty-32-32.map-scen-even/scen-even',
            'empty-32-32.map-scen-random/scen-random',
            'random-32-32-20.map-scen-even/scen-even',
            'random-32-32-20.map-scen-random/scen-random'
                   ]

        result_file.write("file,num_agents,num_agents,prioritized cost,prioritized time,cbs cost,"
                          "cbs generated,cbs expanded,cbs time,disjoint cost,disjoint generated,disjoint expanded,"
                          "disjoint time\n")

        print("***Run All Algorithms***")
        # for dir in folders:
            # for file in sorted(glob.glob(args.instance+dir+'/*')):
        for file in sorted(glob.glob(args.instance + folders[7] + '/*')):
            agents = 2
            done = False
            cbs_no_solution = 0
            disjoint_no_solution = 0
            while agents and not done:
                print("*Running " + file + " with " + str(agents) + " agents")
                done, my_map, starts, goals = import_mapf_instance(file, agents)
                prioritized_solver = PrioritizedPlanningSolver(my_map, starts, goals)
                cbs_solver = CBSSolver(my_map, starts, goals)
                disjoint_solver = CBSSolver(my_map, starts, goals)
                prioritized_node = prioritized_solver.find_solution()
                if cbs_no_solution > 2:
                    cbs_node = cbs_solver.find_solution(False, 1)
                else:
                    cbs_node = cbs_solver.find_solution(False, args.time)
                    if cbs_node['time'] == -1:
                        cbs_no_solution += 1

                if disjoint_no_solution > 2:
                    disjoint_node = disjoint_solver.find_solution(True, 1)
                else:
                    disjoint_node = disjoint_solver.find_solution(True, args.time)
                    if disjoint_node['time'] == -1:
                        disjoint_no_solution += 1

                result_file.write("{},{},{},".format(file, str(len(starts)), agents))
                # if cbs solver can't find solution for the number of agents 3 times in a row then don't attempt search
                # with more agents to save time

                if prioritized_node['time'] == -1:
                    result_file.write("{},{},".format(-1, -1))
                else:
                    result_file.write("{},{},".format(get_sum_of_cost(prioritized_node['paths']),
                                                      prioritized_node['time']))

                result_file.write(
                    "{},{},{},{},{},{},{},{}\n".format(cbs_node['cost'], cbs_node['generated'],
                                                       cbs_node['expanded'], cbs_node['time'],
                                                       disjoint_node['cost'], disjoint_node['generated'],
                                                       disjoint_node['expanded'], disjoint_node['time']))
                agents += 1
    else:
        for file in sorted(glob.glob(args.instance)):

            print("***Import an instance***")
            _, my_map, starts, goals = import_mapf_instance(file, 10)
            print_mapf_instance(my_map, starts, goals)

            if args.solver == "CBS":
                print("***Run CBS***")
                # print("Run " + file)
                cbs = CBSSolver(my_map, starts, goals)
                node = cbs.find_solution(args.disjoint)
                cbs.print_results(node)
                # for path in paths:
                #     print(path)
            elif args.solver == "Independent":
                print("***Run Independent***")
                solver = IndependentSolver(my_map, starts, goals)
                node = solver.find_solution()
            elif args.solver == "Prioritized":
                print("***Run Prioritized***")
                solver = PrioritizedPlanningSolver(my_map, starts, goals)
                node = solver.find_solution()
            else:
                raise RuntimeError("Unknown solver!")

            cost = get_sum_of_cost(node['paths'])
            result_file.write("{},{}\n".format(file, cost))

            if not args.batch:
                print("***Test paths on a simulation***")
                animation = Animation(my_map, starts, goals, node['paths'])
                # animation.save("output.mp4", 1.0)
                animation.show()
    result_file.close()
