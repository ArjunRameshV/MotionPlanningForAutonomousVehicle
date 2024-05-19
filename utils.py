import operator
import math
import heapq

def convert_to_grid_coordinates(i, j, *args):
    x = j * 5 + 2.5  # x-coordinate at the center of the cell
    y = -i * 5 - 2.5   # y-coordinate at the center of the cell
    return (x,y)

def convert_to_list_coordinates(x, y, *args):
    i = int(y/-5)
    j = int(x/5)
    return (i, j)

class StateActionCost:
    def __init__(self, state, action, cost, heuristic=0):
        self.state = tuple(state)
        self.action = action
        self.cost = cost
        self.heuristic = heuristic

    def __lt__(self, obj):
        return (self.cost + self.heuristic) < (obj.cost + obj.heuristic)
    
    def __iter__(self):
        yield self.state
        yield self.action
        yield self.cost
        yield self.heuristic


class SearchAlgorithmTemplate:
    def __init__(self, env):
        self.env = env 
        self.setup()
        
    def setup(self):
        self.start_state_val = tuple(self.env.getStartState())
        self.goal_state_val = tuple(self.env.getGoalState())

        self.start_grid_val = convert_to_grid_coordinates(*self.start_state_val)
        self.goal_grid_val = convert_to_grid_coordinates(*self.goal_state_val)

        self.start_state = StateActionCost((*self.start_grid_val, self.start_state_val[-1]), [], 0, self.compute_heuristic((*self.start_grid_val, self.start_state_val[-1])))
        self.goal_state = StateActionCost((*self.goal_grid_val, self.goal_state_val[-1]), [], 0, self.compute_heuristic((*self.goal_grid_val, self.goal_state_val[-1])))
        
    def compute_heuristic(self, state):
        return 0
    
    def construct_fringe(self):
        '''
        Create a fringe based on the algorithm
        
        Returns
        -------
        fringe : A custom data structure that stores an StateActionCost tuple (state, actions, cost)
        '''
        raise NotImplementedError
    
    def pop_elements_from_fringe(self, fringe):
        '''
        Parameters
        ----------
        fringe : Custom DS based on the algorithm to store StateActionCost

        Returns
        -------
        Pops a (state, actions, cost) tuple based on the algorithm strategy

        '''
        raise NotImplementedError
    
            
    def add_new_element_into_fringe(self, fringe, state_action_update):
        '''
        Based on the algorithm, update the fringe
        
        Parameters
        ----------
        fringe : Custom DS based on the algorithm to store StateActionCost
        state_action_update : A StateActionCost object
        '''
        # add element from a fringe based on algorithm strategy
        # remove duplicate fringe elements if needed
        raise NotImplementedError
    
    def termination_check(self, state):
        i, j = convert_to_list_coordinates(state[0], state[1])
        if i == self.goal_state_val[0] and j == self.goal_state_val[1]:
            return True
        
        return False 

    def search(self):        
        # create a fringe based on the algorithm
        fringe = self.construct_fringe()
        closed_set = set()
        
        while fringe:
            # pop element from a fringe based on algorithm strategy
            state, actions, cost, _ = self.pop_elements_from_fringe(fringe)
            
            # check if we've reached the goal state
            if self.termination_check(state):
                return actions
            
            if state not in closed_set:
                closed_set.add(state)
                
                # get next set of states
                updated_state = list(state)

                successors = self.env.getSuccessors(updated_state)
                for next_state, action, step_cost in successors:
                    # add element to the fringe based on algorithm strategy
                    new_actions = actions + [action]
                    new_cost = cost + step_cost
                    new_heuristic = self.compute_heuristic(next_state)

                    state_action_update = StateActionCost(tuple(next_state), new_actions, new_cost, new_heuristic)
                    self.add_new_element_into_fringe(fringe, state_action_update)
                
        return None  # If no solution found
    