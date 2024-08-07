import math
import heapq

from algo.templates import SearchAlgorithmTemplate
from algo.utils import StateActionCost, convert_to_grid_coordinates, convert_to_list_coordinates

class HybridAStarSearch(SearchAlgorithmTemplate):
    """
    A hybrid A* search algorithm for motion planning in grid-based environments.
    This algorithm combines the traditional A* search with a heuristic function
    that takes into account the vehicle's orientation (pose) in addition to its position.
    """

    def setup(self, beta=0.5):
        self.beta = beta

        self.start_state_val = tuple(self.env.getStartState())
        self.goal_state_val = tuple(self.env.getGoalState())

        self.start_grid_val = convert_to_grid_coordinates(*self.start_state_val)
        self.goal_grid_val = convert_to_grid_coordinates(*self.goal_state_val)

        self.start_state = StateActionCost((*self.start_grid_val, self.start_state_val[-1]), [], 0, self.compute_heuristic((*self.start_grid_val, self.start_state_val[-1])))
        self.goal_state = StateActionCost((*self.goal_grid_val, self.goal_state_val[-1]), [], 0, self.compute_heuristic((*self.goal_grid_val, self.goal_state_val[-1])))

    def termination_check(self, state):
        i, j = convert_to_list_coordinates(state[0], state[1])
        if i == self.goal_state_val[0] and j == self.goal_state_val[1]:
            return True
        return False 

    def compute_heuristic(self, state):
        """
        Compute the heuristic function for the given state.
        The heuristic function is the Euclidean distance between the state
        and the goal state, plus a penalty for the difference in orientation.
        """
        position_diff = (state[0] - self.goal_state_val[0])**2 + (state[1] - self.goal_state_val[1])**2
        orientation_diff = (state[2] - self.goal_state_val[2]) ** 2
        return math.sqrt(position_diff + self.beta * orientation_diff)

    def construct_fringe(self):
        """
        Create a fringe (priority queue) with the initial state.
        """
        return [self.start_state]

    def pop_elements_from_fringe(self, fringe):
        """
        Pop the state with the lowest cost from the fringe.
        """
        return heapq.heappop(fringe)

    def add_new_element_into_fringe(self, fringe, next_state, new_actions, new_cost, new_heuristic):
        """
        Add a new state-action pair to the fringe, replacing the existing pair
        if a shorter path to the same state is found.
        """
        state_action_update = StateActionCost(tuple(next_state), new_actions, new_cost, new_heuristic)
        found = False
        for i, sac in enumerate(fringe):
            if sac.state == state_action_update.state:
                if len(sac.action) > len(state_action_update.action):
                    del fringe[i]
                else:
                    found = True
        if not found:
            heapq.heappush(fringe, state_action_update)
