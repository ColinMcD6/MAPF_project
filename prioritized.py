import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []

        i = 0
        paths_found = True
        while i < self.num_of_agents and paths_found:  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                paths_found = False
                # raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            for agent in range(i + 1, self.num_of_agents):
                time = 0
                for j in range(len(path)):
                    constraints.append({'agent': agent,
                                        'loc': [path[j]],
                                        'time_step': time,
                                        'positive': False})
                    if time != 0:
                        constraints.append({'agent': agent,
                                            'loc': [path[j], path[j - 1]],
                                            'time_step': time,
                                            'positive': False})
                    time += 1
                # Use 'loc': (-1, -1), (x, y) to signify that there is an agent there at all future times
                constraints.append({'agent': agent,
                                    'loc': [(-1, -1), path[len(path) - 1]],
                                    'time_step': time-1,
                                    'positive': False})
            ##############################
            i += 1

        self.CPU_time = timer.time() - start_time

        if paths_found:
            print("\n Found a solution! \n")
            print("CPU time (s):    {:.2f}".format(self.CPU_time))
            print("Sum of costs:    {}".format(get_sum_of_cost(result)))
            print(result)
            return {'paths': result, 'time': self.CPU_time}
        else:
            return {'paths': [], 'time': -1}
