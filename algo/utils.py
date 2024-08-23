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