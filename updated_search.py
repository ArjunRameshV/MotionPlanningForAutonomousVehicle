import operator
import math
import heapq
from collections import deque

from utils import SearchAlgorithmTemplate, StateActionCost, convert_to_list_coordinates

class AStartSearch(SearchAlgorithmTemplate):
  """
  Your search algorithm needs to return a list of actions that reaches the goal
  Strategy: Search the deepest nodes in the search tree first
  """
  def compute_heuristic(self, state):
      return math.sqrt((state[0] - self.goal_state_val[0])**2 + (state[1] - self.goal_state_val[1])**2)
    
  def construct_fringe(self):
      return [self.start_state]
  
  def pop_elements_from_fringe(self, fringe):
      return heapq.heappop(fringe)
  
  def add_new_element_into_fringe(self, fringe, state_action_update):
      found = False
      for i, sac in enumerate(fringe):
          if sac.state == state_action_update.state:
              if len(sac.action) > len(state_action_update.action):
                  del fringe[i]
              else:
                  found = True
      if not found:
          heapq.heappush(fringe, state_action_update)


class HybridAStarSearch(SearchAlgorithmTemplate):
    """
    A hybrid A* search algorithm for motion planning in grid-based environments.
    This algorithm combines the traditional A* search with a heuristic function
    that takes into account the vehicle's orientation (pose) in addition to its position.
    """
    beta = 0.5

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

    def add_new_element_into_fringe(self, fringe, state_action_update):
        """
        Add a new state-action pair to the fringe, replacing the existing pair
        if a shorter path to the same state is found.
        """
        found = False
        for i, sac in enumerate(fringe):
            if sac.state == state_action_update.state:
                if len(sac.action) > len(state_action_update.action):
                    del fringe[i]
                else:
                    found = True
        if not found:
            heapq.heappush(fringe, state_action_update)
