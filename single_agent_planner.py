import heapq


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        path_length = len(path)
        rst += path_length - 1
        # subtract from cost if end of path location is repeated
        i = -1
        while path_length + i > 0 and path[i] == path[i-1]:
            rst -= 1
            i -= 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constructs the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    # Task 4.1: Allow positive constraints. An agent is required to be in a cell
    #               At a given time step. All other agents are therefore not allowed
    #               to be in that cell at that time.

    constraint_table = {}

    for constraint in constraints:
        if constraint['positive'] and constraint['agent'] != agent:
            if constraint['time_step'] not in constraint_table:
                constraint_table.update({constraint['time_step']: [[constraint['loc'], False]]})
            else:
                constraint_table[constraint['time_step']].append([constraint['loc'], False])
        elif constraint['agent'] == agent:
            if constraint['time_step'] not in constraint_table:
                constraint_table.update({constraint['time_step']: [[constraint['loc'], constraint['positive']]]})
            else:
                constraint_table[constraint['time_step']].append([constraint['loc'], constraint['positive']])
    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def get_path_length(last_node):
    path = []
    curr = last_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    return len(path)


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    if is_vertex_constrained(next_loc, next_time, constraint_table) \
            or is_edge_constrained(curr_loc, next_loc, next_time, constraint_table)\
            or finished_agent_in_way(next_loc, next_time, constraint_table):
        return True
    else:
        return False


def is_vertex_constrained(next_loc, next_time, constraint_table):
    # Check for positive constraint at next_time. If there is and next_loc does not equal the positive
    # constraint location then vertex is constrained
    if next_time in constraint_table:
        for loc in constraint_table[next_time]:
            if loc[0] != [next_loc] and loc[1]:
                return True
        if [[next_loc], False] in constraint_table[next_time]:
            return True
        return False
    else:
        return False


def is_edge_constrained(curr_loc, next_loc, next_time, constraint_table):
    if next_time in constraint_table:
        for loc in constraint_table[next_time]:
            if loc[0] != [curr_loc, next_loc] and loc[1]:
                return True
        if [[curr_loc, next_loc], False] in constraint_table[next_time]:
            return True
        return False
    else:
        return False


def finished_agent_in_way(next_loc, next_time, constraint_table):
    for i in range(next_time):
        if i in constraint_table and [(-1, -1), next_loc] in constraint_table[i]:
            return True
    return False


def no_goal_constraints(time, goal, constraint_table):
    max_time = max_time_of_constraints(constraint_table)
    if time <= max_time:
        for t in range(time, max_time+1):
            if is_vertex_constrained(goal, t, constraint_table):
                return False
    return True


def max_time_of_constraints(constraint_table):
    if len(constraint_table.keys()) == 0:
        return -1
    else:
        return max(constraint_table.keys())


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def outside_boundary(loc, width, height):
    if loc[0] < 0 or loc[1] < 0 or loc[0] >= width or loc[1] >= height:
        return True
    return False


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each time_step
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    max_path_length = len(my_map[0])*len(my_map)/2
    open_list = []
    closed_list = dict()
    earliest_goal_time_step = 0
    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints, agent)
    root = {'loc': start_loc,
            'time_step': 0,
            'g_val': 0,
            'h_val': h_value,
            'parent': None}
    push_node(open_list, root)
    closed_list[(root['loc'], root['time_step'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc:
            print('hello')
        if curr['loc'] == goal_loc and no_goal_constraints(curr['time_step'], curr['loc'], constraint_table):
            return get_path(curr)
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if outside_boundary(child_loc, len(my_map), len(my_map[0])) or my_map[child_loc[0]][child_loc[1]]:
                continue
            if is_constrained(curr['loc'], child_loc, curr['time_step'] + 1, constraint_table):
                continue
            child = {'loc': child_loc,
                     'time_step': curr['time_step'] + 1,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr}
            if (child['loc'], child['time_step']) in closed_list:
                existing_node = closed_list[(child['loc'], child['time_step'])]
                if compare_nodes(child, existing_node) and get_path_length(child) < max_path_length:
                    closed_list[(child['loc'], child['time_step'])] = child
                    push_node(open_list, child)
            elif get_path_length(child) < max_path_length:
                closed_list[(child['loc'], child['time_step'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
