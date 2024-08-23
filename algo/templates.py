

class SearchAlgorithmTemplate:
    def __init__(self, env, *args, **kwargs):
        self.env = env 
        self.setup(*args, **kwargs)
        
    def setup(self, *args, **kwargs):
        '''
        Setting up the algorithm variables based on the map and computation logic 
        '''
        pass
        
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
    
            
    def add_new_element_into_fringe(self, fringe, next_state, new_actions, new_cost, new_heuristic):
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
        '''
        Function to check whether the algorithm should terminate
        '''
        raise NotImplementedError
    
    def search(self):
        '''
        A generic search logic that can be used to implement different search algorithms 
        '''
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

                    self.add_new_element_into_fringe(fringe, next_state, new_actions, new_cost, new_heuristic)
                
        return None  # If no solution found
    