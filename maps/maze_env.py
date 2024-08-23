import copy
import maps.custom_maze_maps as custom_maze_maps
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

import math
from algo.utils import convert_to_grid_coordinates, convert_to_list_coordinates

#Flag to enable plots
enable_plots  = False

class Maze:
    """
    This class outlines the structure of the maze problem
    """
    
    maze_map = [] # To store map data, start and goal points

    # [delta_x, delta_y, description]
    four_neighbor_actions = {
        'l1': 10,
        's': 0,
        'r1': -10,
    }

    v = 12  # Velocity in m/s
    L = 5  # Car length in meters
    dt = 0.5  # Time step in seconds
    
    # Legal moves

    if enable_plots:
        #Setup plot
        plt.close('all')
        map_plot_copy = []
        fig,ax = plt.subplots(1)
        plt.axis('equal')
    
    def plot_map(self):
        """
        Plot
        """
        if enable_plots:
            start = self.getStartState()
            goal = self.getGoalState()
            self.map_plot_copy[start[0]][start[1]] = custom_maze_maps.start_id
            self.map_plot_copy[goal[0]][goal[1]] = custom_maze_maps.goal_id
            plt.imshow(self.map_plot_copy, cmap=plt.cm.tab20c, norm=self.plot_colormap_norm)
            plt.show()
        
    # default constructor
    def __init__(self, id):
        """
        Sets the map as defined in file custom_maze_maps
        """
        #Set up the map to be used
        self.maze_map = custom_maze_maps.maps_dictionary[id]
        if enable_plots:
            self.map_plot_copy = copy.deepcopy(self.maze_map.map_data)
            self.plot_map()
        return
        
    def getStartState(self):
        """
        Returns the start state for the search problem 
        """
        start_state = self.maze_map.start
        return start_state
    
    def getGoalState(self):
        """
        Returns the start state for the search problem 
        """
        goal_state =  self.maze_map.goal
        return goal_state
        
    def isGoalState(self, state):
        """
        state: Search state
        
        Returns True if and only if the state is a valid goal state
        """
        goal_state = self.getGoalState()
        if state[0] == goal_state[0] and state[1] == goal_state[1]:
            return True
        else:
            return False
    
    def bicycle_model(self, x, y, theta, delta, v, L, dt):
        # Update car's position and heading using bicycle model equations
        x_new = round(x + v * np.cos(theta) * dt, 4)
        y_new = round(y + v * np.sin(theta) * dt, 4)
        theta_new = round(theta + (v * np.tan(np.radians(delta))) / L * dt, 4)
        return x_new, y_new, theta_new
    
    def collision_checker(self, state):
        i, j = convert_to_list_coordinates(state[0], state[1])
        # print(state, i, j)
        try:
            if self.maze_map.map_data[i][j] == custom_maze_maps.obstacle_id:
                return True
        except IndexError:
            pass

        return False
    
    def cost_generator(self, state, action):
        turn_cost_factor = 0.2 if action in ['l1', 'r1'] else 0

        i, j = convert_to_list_coordinates(state[0], state[1])
        if self.maze_map.map_data[i][j] == custom_maze_maps.free_space_id2:
            return custom_maze_maps.free_space_id2_cost * (1 + turn_cost_factor)
        return custom_maze_maps.free_space_id1_cost * (1 + turn_cost_factor)
        

    def getSuccessors(self, state):
        """
        state: Search state
        
        For a given state, this should return a list of triples, 
        (successor, action, stepCost), where 'successor' is a 
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental 
        cost of expanding to that successor
        """
        if enable_plots:
            #Update changes on the plot copy
            self.map_plot_copy[state[0]][state[1]] = custom_maze_maps.expanded_id
        
        successors = []
        for action in self.four_neighbor_actions:
            
            #Get indiivdual action
            x_new, y_new, theta_new = self.bicycle_model(*state, self.four_neighbor_actions.get(action), self.v, self.L, self.dt)
            
            #Get successor
            new_successor = [x_new, y_new, theta_new]
            new_action = action
            
            # Check for obstacle 
            if self.collision_checker(new_successor):
                continue
            
            if enable_plots:
                #Update changes on the plot copy
                if self.map_plot_copy[new_successor[0]][new_successor[1]] != custom_maze_maps.expanded_id:
                    self.map_plot_copy[new_successor[0]][new_successor[1]] = custom_maze_maps.fringe_id
            
            #Check cost
            new_cost = self.cost_generator(new_successor, action)
                
            successors.append([new_successor, new_action, new_cost])
        
        if enable_plots:
            #Plot the changes
            self.plot_map()    
        return successors
