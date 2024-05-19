import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import custom_maze_maps

import maze_env
import numpy as np
from scipy.interpolate import splprep, splev

import operator
import math
import heapq

from updated_search import HybridAStarSearch
from sample_search import RRT

def convert_to_grid_coordinates(i, j):
    x = j * 5 + 2.5  # x-coordinate at the center of the cell
    y = -i * 5 - 2.5   # y-coordinate at the center of the cell
    return (x,y)

def convert_to_list_coordinates(x, y):
    i = int(math.floor(y/-5))
    j = int(math.floor(x/5))
    return (i, j)

def bicycle_model(x, y, theta, delta, v, L, dt):
    # Update car's position and heading using bicycle model equations
    x_new = x + v * np.cos(theta) * dt
    y_new = y - v * np.sin(theta) * dt
    theta_new = theta + (v * np.tan(np.radians(delta))) / L * dt
    return x_new, y_new, theta_new

def generate_grid_world(grid):
    grid_world = []
    for i, row in enumerate(grid):
        grid_row = []
        for j, cell in enumerate(row):
            if cell == 16:  # obstacle
                grid_row.append(None)  # represent obstacle with None
            else:  # free space
                x, y = convert_to_grid_coordinates(i, j)
                grid_row.append((x, y))
        grid_world.append(grid_row)
    return grid_world

def visualize_grid_world(grid_world, start_pos, goal_pos, car_poses=None, heading_angles=None, converted=False):
    fig, ax = plt.subplots(figsize=(8, 6))
    for i, row in enumerate(grid_world):
        for j, cell in enumerate(row):
            if cell is not None:
                # Plot blue dot
                ax.plot(cell[0], cell[1], 'bo', markersize=5)
                # Plot square boundary around blue dot
                square_boundary = [
                    (cell[0] - 2.5, cell[1] - 2.5),  # bottom-left corner
                    (cell[0] + 2.5, cell[1] - 2.5),  # bottom-right corner
                    (cell[0] + 2.5, cell[1] + 2.5),  # top-right corner
                    (cell[0] - 2.5, cell[1] + 2.5),  # top-left corner
                    (cell[0] - 2.5, cell[1] - 2.5),  # bottom-left corner (to close the square)
                ]
                square_boundary_x, square_boundary_y = zip(*square_boundary)
                ax.plot(square_boundary_x, square_boundary_y, 'b-', linewidth=1)  # Plot square boundary
            else:
                # Plot red rectangle for obstacles
                x = j * 5  # x-coordinate of bottom-left corner of rectangle
                y = -i * 5  # y-coordinate of bottom-left corner of rectangle
                rect = Rectangle((x, y), 5, -5, color='red', alpha=0.5)
                ax.add_patch(rect)

    if start_pos is not None:
        start_x, start_y = convert_to_grid_coordinates(*start_pos)
        ax.plot(start_x, start_y, 'gx', markersize=12)  # Plot start state as green 'x'

    if goal_pos is not None:
        goal_x, goal_y = convert_to_grid_coordinates(*goal_pos)
        ax.plot(goal_x, goal_y, 'y*', markersize=12)  # Plot goal state as yellow star

    for car_pos, heading_angle in zip(car_poses, heading_angles):
        if car_pos is not None:
            if converted:
                car_x, car_y = car_pos
                # Plot car as a rectangle
                car_rect = plt.Rectangle((car_x - 2, car_y - 2), 4, 4, angle=heading_angle, color='green', alpha=0.5)
                ax.add_patch(car_rect)
                # Plot arrow representing heading direction
                arrow_length = 3
                dx = arrow_length * np.cos(heading_angle)
                dy = arrow_length * np.sin(heading_angle)
                ax.arrow(car_x + dx/2, car_y + dy/2, dx/2, dy/2, head_width=0.5, head_length=1, fc='k', ec='k')

                # Connect car poses using splines
                if len(car_poses) > 1:
                    car_poses_x = [(car_pos[0]) for car_pos in car_poses]
                    car_poses_y = [(car_pos[1]) for car_pos in car_poses]
                    tck, u = splprep([car_poses_x, car_poses_y], s=0)
                    u_new = np.linspace(u.min(), u.max(), 1000)
                    spline_x, spline_y = splev(u_new, tck)
                    ax.plot(spline_x, spline_y, 'r--', alpha=0.8)  # Plot spline connecting car poses

            else:
                car_x, car_y = convert_to_grid_coordinates(*car_pos)
                # Plot car as a rectangle
                car_rect = plt.Rectangle((car_x - 2, car_y - 2), 4, 4, angle=heading_angle, color='green', alpha=0.5)
                ax.add_patch(car_rect)
                # Plot arrow representing heading direction
                arrow_length = 3
                dx = arrow_length * np.cos(heading_angle)
                dy = arrow_length * np.sin(heading_angle)
                ax.arrow(car_x + dx/2, car_y + dy/2, dx/2, dy/2, head_width=0.5, head_length=1, fc='k', ec='k')

                # Connect car poses using splines
                if len(car_poses) > 1:
                    car_poses_x = [(car_pos[1] * 5 + 2.5) for car_pos in car_poses]
                    car_poses_y = [(-car_pos[0] * 5 - 2.5) for car_pos in car_poses]
                    tck, u = splprep([car_poses_x, car_poses_y], s=0)
                    u_new = np.linspace(u.min(), u.max(), 1000)
                    spline_x, spline_y = splev(u_new, tck)
                    ax.plot(spline_x, spline_y, 'r--', alpha=0.8)  # Plot spline connecting car poses
    
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Grid World')
    plt.show()

current_maze = maze_env.Maze(8)
start_pos = current_maze.getStartState()  # Start state coordinates
goal_pos = current_maze.getGoalState()   # Goal state coordinates

############################ The Algo ############################

algo = HybridAStarSearch(current_maze)
path = algo.search()

angles = []
changes = []
coords = []
if path:
    # Check path validity
    row,col, theta = current_maze.getStartState()
    x,y = convert_to_grid_coordinates(row, col)
    for action in path:
        
        changes.append(current_maze.four_neighbor_actions.get(action))

        x, y, theta = current_maze.bicycle_model(x, y, theta, current_maze.four_neighbor_actions.get(action), current_maze.v, current_maze.L, current_maze.dt)
        # print(x, y, theta)
        coords.append((x,y))
        angles.append(theta)

    if algo.termination_check((x, y, theta)):
        print('Found a path of %d moves' % (len(path))) 
        print(coords)
        print(angles)
        print(path)
        print(changes)
    else:
        print('Not a valid path')
    
else:        
    print("Could not find a path")

start_state = current_maze.getStartState()
goal_state = current_maze.getGoalState()
start_pos = (start_state[0], start_state[1])  # Start state coordinates
goal_pos = (goal_state[0], goal_state[1])   # Goal state coordinates

grid_world = generate_grid_world(current_maze.maze_map.map_data)
visualize_grid_world(grid_world, start_pos, goal_pos, coords, angles, True)



