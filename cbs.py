import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    collision = None
    v_collision = detect_vertex_collision(path1, path2)
    e_collision = detect_edge_collision(path1, path2)
    if v_collision is not None:
        if e_collision is not None:
            if v_collision['time_step'] < e_collision['time_step']:
                collision = v_collision
            else:
                collision = e_collision
        else:
            collision = v_collision
    elif e_collision is not None:
        collision = e_collision
    return collision


def detect_vertex_collision(path1, path2):
    collision = None
    longest_path = max(len(path1), len(path2))
    t = 0
    collision_found = False
    while t < longest_path and not collision_found:
        loc1 = get_location(path1, t)
        loc2 = get_location(path2, t)
        if loc1 == loc2:
            collision_found = True
            collision = {'loc': [loc1], 'time_step': t}
        t += 1
    return collision


def detect_edge_collision(path1, path2):
    collision = None
    longest_path = max(len(path1), len(path2))
    t = 1
    collision_found = False
    while t < longest_path and not collision_found:
        start_loc1 = get_location(path1, t - 1)
        start_loc2 = get_location(path2, t - 1)
        end_loc1 = get_location(path1, t)
        end_loc2 = get_location(path2, t)
        if start_loc1 == end_loc2 and start_loc2 == end_loc1:
            collision_found = True
            collision = {'loc': [start_loc1, end_loc1], 'time_step': t}
        t += 1
    return collision


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    # latest_time = max(len(p) for p in paths)
    # for t in range(0, latest_time):
    for i in range(0, len(paths)):
        for j in range(i + 1, i + len(paths)):
            collision = detect_collision(paths[i], paths[j % len(paths)])
            if collision is not None:
                if ({'a1': j % len(paths), 'a2': i, 'loc': collision['loc'],
                     'time_step': collision['time_step']}) not in collisions:
                    collisions.append(
                        {'a1': i, 'a2': j % len(paths), 'loc': collision['loc'], 'time_step': collision['time_step']})
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    agent1 = collision['a1']
    agent2 = collision['a2']
    loc = collision['loc']
    t = collision['time_step']
    if len(loc) > 1:  # edge collision
        constraint = [{'agent': agent1, 'loc': [loc[0], loc[1]], 'time_step': t, 'positive': False},
                      {'agent': agent2, 'loc': [loc[1], loc[0]], 'time_step': t, 'positive': False}]
    else:
        constraint = [{'agent': agent1, 'loc': loc, 'time_step': t, 'positive': False},
                      {'agent': agent2, 'loc': loc, 'time_step': t, 'positive': False}]
    return constraint


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    agent_index = random.randint(0, 1)
    agents = [collision['a1'], collision['a2']]
    loc = collision['loc']
    t = collision['time_step']
    if len(loc) > 1:  # edge collision
        constraint = [{'agent': agents[agent_index], 'loc': [loc[agent_index], loc[(agent_index+1) % 2]],
                       'time_step': t, 'positive': False},
                      {'agent': agents[agent_index], 'loc': [loc[agent_index], loc[(agent_index+1) % 2]],
                       'time_step': t, 'positive': True}]
    else:  # vertex collision
        constraint = [{'agent': agents[agent_index], 'loc': loc, 'time_step': t, 'positive': False},
                      {'agent': agents[agent_index], 'loc': loc, 'time_step': t, 'positive': True}]
    return constraint


def paths_violate_constraint(constraint, paths):
    if not constraint['positive']:
        return [constraint['agent']]

    affected_agents = []
    for agent in range(0, len(paths)):
        if agent != constraint['agent'] and constraint['time_step'] < len(paths[agent]):
            curr_loc = paths[agent][constraint['time_step']]
            if ([curr_loc] == constraint['loc'] or
                    (constraint['time_step'] > 0 and
                     [curr_loc, paths[agent][constraint['time_step']-1]] == constraint['loc'])):
                affected_agents.append(agent)
        elif agent != constraint['agent'] and [paths[agent][len(paths[agent])-1]] == constraint['loc']:
            affected_agents.append(agent)
    return affected_agents


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        # print("Generate node {}".format(self.num_of_generated))
        # for path in node['paths']:
        #     print(path)
        # for constraint in node['constraints']:
        #     print(constraint)
        # print(node['cost'])
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        # print("Expand node {}".format(id))
        # print("Expand node {}".format(id))
        # for path in node['paths']:
        #     print(path)
        # for constraint in node['constraints']:
        #     print(constraint)
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        # print(root['collisions'])

        # Task 3.2: Testing
        # for collision in root['collisions']:
        #     print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        while len(self.open_list) > 0:
            node = self.pop_node()
            collisions = node['collisions']
            # print(node)
            if len(collisions) == 0:
                # self.print_results(node)
                # for cons in node['constraints']:
                #     print(cons)
                # return node['paths']
                # noinspection PyPep8Naming
                CPU_time = timer.time() - self.start_time
                node['time'] = CPU_time
                node['generated'] = self.num_of_generated
                node['expanded'] = self.num_of_expanded
                return node
            collision = collisions[0]
            if disjoint:
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)
            for constraint in constraints:
                all_constraints = []
                # deep copy of node's constraints
                for old_constraint in node['constraints']:
                    all_constraints.append(old_constraint)
                all_constraints.append(constraint)
                paths = []
                # deep copy of node's paths
                for old_path in node['paths']:
                    paths.append(old_path)
                new_node = {'cost': 0, 'constraints': all_constraints, 'paths': paths,
                            'collisions': []}
                affected_agents = paths_violate_constraint(constraint, new_node['paths'])
                i = 0
                no_path = False
                while i < len(affected_agents) and not no_path:
                    path = a_star(self.my_map, self.starts[affected_agents[i]], self.goals[affected_agents[i]],
                                  self.heuristics[affected_agents[i]], affected_agents[i], new_node['constraints'])
                    if path is not None:
                        new_node['paths'][affected_agents[i]] = path
                    else:
                        no_path = True
                    i += 1
                if not no_path:
                    new_node['collisions'] = detect_collisions(new_node['paths'])
                    new_node['cost'] = get_sum_of_cost(new_node['paths'])
                    self.push_node(new_node)

        return None  # Failed to find solutions

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
