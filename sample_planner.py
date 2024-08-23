import math
from algo.hybrid_a_star import HybridAStarSearch
from algo.utils import convert_to_grid_coordinates
from gps_processing.utils import update_gps_readings
import maps.maze_env as maze_env


class Planner:
    def __init__(self):
        self.segment_id = None
        self.prev_segment_id = None
        self.local_planner_activated = False
        self.steering_inputs = []
        self.frame = 1
    
    segmented_coordinates = {
        "global_straight_one": {
            0: None, # x coordinate range
            1: [3, 7]  # y coordinate range
        },
        "local_algo_one":{
            0:  None,
            1:  [-9, -7],
        },
        "global_curve_one": {
            0: None,
            1: [-66, -63],
        },
        "global_straight_two": {
            0: [-75, -56],
            1: [-106, -96],
        },
        "local_algo_two": {
            0: [-94, -90],
            1: [-93, -88]
        },
        "global_termination": {
            0: [-56, -40],
            1: [-110, -97]
        }
    }
    
    segmented_planning = {
        "global_straight_one": {
            "speed": 12,
            "steering_angle": 0, # degrees
            "duration": 50,
            "segment_compass_index": 1,
            "compute_algo": False
        },
        "local_algo_one": {
            "maze": maze_env.Maze(7),
            "speed": 12,
            "compute_algo": True,
            "algo_done": False,
        },
        "global_curve_one": {
            "speed": 12,
            "steering_angle": -5, # degrees
            "duration": 21,
            "compute_algo": False
        },
        "global_straight_two": {
            "speed": 12,
            "steering_angle": 0, # degrees
            "duration": 10,
            "segment_compass_index": 0,
            "compute_algo": False
        },
        "local_algo_two": {
            "maze": maze_env.Maze(8),
            "speed": 12,
            "compute_algo": True,
            "algo_done": False,
        },
        "global_termination": {
            "speed": 0,
            "steering_angle": 0, # degrees
            "duration": 10,
            "segment_compass_index": 0,
            "compute_algo": False
        }
    }

    def compute_hybrid_a_star_path(self, current_maze):
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
                coords.append((x,y))
                angles.append(theta)
        else:        
            print("Could not find a path")

        return changes

    def check_waypont_reached(self, segment_coordinates_range, x, y):
        '''
        Determine whether the vehicle has reached a particular segment, 
        is done by checking the whether gps coordinates aligns with segment coordinates 
        
        An example of segment_coordinates_range
        {
            0: [-94, -90],
            1: [-93, -88]
        }
        '''
        x_check = False
        y_check = False
        if segment_coordinates_range.get(0):
            if segment_coordinates_range.get(0)[0] <= x <= segment_coordinates_range.get(0)[1]:
                x_check = True
        else:
            x_check = True

        if segment_coordinates_range.get(1):
            if segment_coordinates_range.get(1)[0] <= y <= segment_coordinates_range.get(1)[1]:
                y_check = True
        else:
            y_check = True

        return x_check and y_check
    
    def take_heuristic_step(self, gps_coords):
        if not self.steering_inputs:
            self.steering_inputs = self.compute_hybrid_a_star_path(self.segmented_planning[self.segment_id]["maze"])
            print(self.steering_inputs)
            return (0, None)
        else:
            if self.frame < len(self.steering_inputs):
                # print(f"Current speed: {self.segmented_planning[segment_id]['speed']}")
                self.frame += 1
                return (
                    self.segmented_planning[self.segment_id]["speed"],
                    math.radians(-1 * self.steering_inputs[self.frame - 1]//2)
                )
            else:
                # TODO: Determine how to continue the previous algorithm
                print("--------------")
                print(f"The gps coordinates are: ({gps_coords[0]}, {gps_coords[1]})")
                print(f"!!!! We are switching back to {self.prev_segment_id} planner from {self.segment_id} !!!!")
                self.segmented_planning[self.segment_id]["algo_done"] = True
                self.steering_inputs = []
                if self.prev_segment_id:
                    self.segment_id = self.prev_segment_id
        return (None, None)

    def take_planned_step(self, compass_value):
        # self.set_speed(self.segmented_planning[self.segment_id]["speed"])
        steering_angle = None
        if "segment_compass_index" in self.segmented_planning[self.segment_id]:
            # print(f"Performing deviation correction as per {segment_id} planner")
            current_deviation = compass_value[self.segmented_planning[self.segment_id]["segment_compass_index"]]
            if abs(current_deviation) >= 0.1:
                steering_angle = current_deviation
                # print(f"Performing deviation correction as per {segment_id} planner by {current_deviation} radians")
            else:
                steering_angle = math.radians(self.segmented_planning[self.segment_id]["steering_angle"])
                # print(f"Performing deviation correction as per {segment_id} planner by {math.radians(self.segmented_planning[segment_id]['steering_angle'])} radians")
        else:
            steering_angle = math.radians(self.segmented_planning[self.segment_id]["steering_angle"])

        return (
            self.segmented_planning[self.segment_id]["speed"],
            steering_angle
        )

    def update_segments(self, waypoint_ind, gps_coords):
        self.prev_segment_id = self.segment_id
        self.segment_id = waypoint_ind
        if  self.prev_segment_id != self.segment_id:
            print("------------------------------ Segment Changed --------------------------------")
            print(f"The gps coordinates are: ({gps_coords[0]}, {gps_coords[1]})")
            print(f"!!!! Motion Planning will be performed using: {self.segment_id} !!!!")
    
    def move(self, gps_coords, compass_value):
        '''
        This is the function that determines what action must be done by the vehicle during each 'actionable' timestep

        This planner uses a set of waypoints generated from the webots environment to determine how the vehicle will behave
        There are two types of abstract behaviours the vehicles tends to follow, using a local or global planning strategy.

        The local planner segment will result in triggering a heuristics algorithm to find a path for the vehicle and the 
        global planner segment will result in the vehicle following a well defined strategy.

        * Every waypoint segment will be associated with a global planner
        * The waypoint segment can also have a local planner, in case we've placed objects along the path
        * The local planner would stop the vehicle, plan and execute a safe route, and then finally switch control back to the global planner
        
        We need to keep track of the current global planner and also determine if the context switch to local planner has occured
        '''
        if gps_coords is not None:
            x, y , _ = gps_coords
            print(f"current segment id: {self.segment_id}")
            for waypoint_ind in self.segmented_coordinates:
                # we iterate over all the stored waypoints segments to determine whether a new waypoint has been reached
                # this is done by determining whether the current gps coords are within the predetermined gps segments for a waypoint strip

                if self.segment_id and "algo" in self.segment_id and not self.segmented_planning[self.segment_id]["algo_done"]:
                    # Wait till the algorithm step is done before sampling new instructions
                    break
                
                segment_coordinates_range = self.segmented_coordinates[waypoint_ind]
                if self.check_waypont_reached(segment_coordinates_range, x, y):
                    # once a waypoint is reached, make sure to update the global segment reference
                    self.frame = 0
                    self.update_segments(waypoint_ind, gps_coords)

        if self.segment_id:
            if "termination" in self.segment_id:
                print("!!!! MOTION PLANNING COMPLETED !!!!")
                print('------------------------------------------------------------------------------------------------')
                return (-1, -1)

            if self.segmented_planning[self.segment_id]["compute_algo"] and not self.segmented_planning[self.segment_id]["algo_done"]:
                    return self.take_heuristic_step(gps_coords)
           
            return self.take_planned_step(compass_value)
        
        return (None, None)  # ignored