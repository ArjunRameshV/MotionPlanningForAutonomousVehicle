# Import necessary modules
import math
import sys


from config_manager import ConfigManager
from controller import Camera, Display, GPS, Keyboard, Lidar, Robot, Supervisor
from manual_controller import ManualControllerMixin
from sensors import SensorsMixin
from vehicle import Driver, Car

from enum import Enum

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import numpy as np
import threading
import polars as pl

import maze_env
from algo.a_star import HybridAStarSearch
from algo.utils import convert_to_grid_coordinates

from utilities.env import generate_grid_world, visualize_grid_world

class Coordinate(Enum):
    X = 0
    Y = 1
    Z = 2

class AVDriver(Car, ManualControllerMixin, SensorsMixin):
    def __init__(self):
        super().__init__()
        self.keyboard = Keyboard()
        self.config = ConfigManager()
        # self.init_constants()

        # WbDeviceTag gps;
        self.gps_coords = self.config["sensors"]["gps"]["init_coords"]
        self.gps_speed = self.config["sensors"]["gps"]["init_speed"]

        # misc variables
        self.speed = self.config["vehicle_settings"]["initial_speed"]
        self.steering_angle = self.config["vehicle_settings"]["initial_steering_angle"]
        self.manual_steering = self.config["vehicle_settings"]["manual_steering"]
        self.autodrive = self.config["vehicle_settings"]["autodrive_enabled"]

        # set driver configs
        self.setHazardFlashers(True)
        self.setDippedBeams(True)
        self.setAntifogLights(True)
        self.setWiperMode(Driver.SLOW)

        # allow to switch to manual control
        self.keyboard.enable(self.config["TIME_STEP"])

    # ----------------- HELPER FUNCTIONS ----------------- #

    # set target speed
    def set_speed(self, kmph):
        # max speed
        if kmph > 50.0:
            kmph = 50.0

        self.speed = kmph
        self.setCruisingSpeed(kmph)

    def print_infomatics(self):
        print("You can drive this car!")
        print("Select the 3D window and then use the cursor keys to:")
        print("[LEFT]/[RIGHT] - steer")
        print("[UP]/[DOWN] - accelerate/slow down")
    
    def compute_gps_speed(self):
        coords = self.gps.getValues()
        speed_ms = self.gps.getSpeed()
        
        self.gps_speed = speed_ms * 3.6  # convert from m/s to km/h
        self.gps_coords = coords[:]

    # positive: turn right, negative: turn left
    def set_steering_angle(self, wheel_angle):
        # limit the difference with previous steering_angle
        if wheel_angle - self.steering_angle > 0.1:
            wheel_angle = self.steering_angle + 0.1
        if wheel_angle - self.steering_angle < -0.1:
            wheel_angle = self.steering_angle - 0.1
        
        new_angle = self.steering_angle + wheel_angle
        
        # limit range of the steering angle
        if new_angle > 0.5:
            new_angle = 0.5
        elif new_angle < -0.5:
            new_angle = -0.5
        
        self.setSteeringAngle(new_angle)

    def set_autodrive(self, onoff):
        if self.autodrive != onoff:
            self.autodrive = onoff

    # ----------------- DISPLAY FUNCTION ----------------- #
                
    def update_display(self):
        NEEDLE_LENGTH = 50.0

        # display background
        self.display.imagePaste(self.speedometer_image, 0, 0, False)

        # draw speedometer needle
        current_speed = self.getCurrentSpeed()
        if math.isnan(current_speed):
            current_speed = 0.0
        alpha = current_speed / 260.0 * 3.72 - 0.27
        x = -NEEDLE_LENGTH * math.cos(alpha)
        y = -NEEDLE_LENGTH * math.sin(alpha)
        self.display.drawLine(100, 95, 100 + x, 95 + y)

        # draw text
        txt = f"GPS coords: {self.gps_coords[Coordinate.X.value]:.1f} {self.gps_coords[Coordinate.Z.value]:.1f}"
        self.display.drawText(txt, 10, 130)
        txt = f"GPS speed:  {self.gps_speed:.1f}"
        self.display.drawText(txt, 10, 140)

    # ----------------- CORE FUNCTION ----------------- #

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

            # if algo.termination_check((x, y, theta)):
            #     print('Found a path of %d moves' % (len(path))) 
            #     print(coords)
            #     print(angles)
            #     print(path)
            #     print(changes)
            # else:
            #     print('Not a valid path')
        else:        
            print("Could not find a path")

        return changes

    def check_waypont_reached(self, segmented_x_coordinates, segmented_y_coordinates, x, y, segmented_planning, segment_index):
        if segment_index and "algo" in segment_index and not segmented_planning[segment_index]["algo_done"]:
            # Wait till the algorithm step is done before sampling new instructions
            return False

        x_check = False
        y_check = False
        if segmented_x_coordinates:
            if segmented_x_coordinates[0] <= x <= segmented_x_coordinates[1]:
                x_check = True
        else:
            x_check = True

        if segmented_y_coordinates:
            if segmented_y_coordinates[0] <= y <= segmented_y_coordinates[1]:
                y_check = True
        else:
            y_check = True

        return x_check and y_check

    def main_loop(self):

        segmented_coordinates = {
            "global_straight_one": {
                0: None, # x coordinate range
                1: [3, 5]  # y coordinate range
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
                "deviation_check_index": 1,
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
                "deviation_check_index": 0,
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
                "deviation_check_index": 0,
                "compute_algo": False
            }
        }

        # main loop
        i = 1
        local_i = 1
        current_steering_input = []
        prev_segment_index = None
        segment_index = None
        self.gps_stream = []
        print('------------------------------------------------------------------------------------------------')
        while self.step() != -1:
            # get user input
            self.check_keyboard()
            # updates sensors only every self.config["TIME_STEP"] milliseconds
            self.config["TIME_STEP"] = 1000
            if i % (int(self.config["TIME_STEP"] / self.getBasicTimeStep())) == 0:
                if self.config["sensors"]["camera"]["enabled"]:
                    # capture the front camera image
                    self.camera.getImage()

                # update sensor data
                if self.config["sensors"]["gps"]["enabled"]:
                    self.compute_gps_speed()
                    # print(f"The gps coordinates are: ({self.gps_coords[0]}, {self.gps_coords[1]})")
                    # print(self.compass.getValues())

                if self.gps_coords is None:
                    self.set_speed(0)
                    self.set_steering_angle(math.radians(-10))
                else:
                    x, y , _ = self.gps_coords
                    for waypoint_ind in segmented_coordinates:
                        segmented_x_coordinates = segmented_coordinates[waypoint_ind].get(0)
                        segmented_y_coordinates = segmented_coordinates[waypoint_ind].get(1)

                        if self.check_waypont_reached(segmented_x_coordinates, segmented_y_coordinates, x, y, segmented_planning, segment_index):
                            prev_segment_index = segment_index
                            segment_index = waypoint_ind
                            local_i = 0
                            if prev_segment_index != segment_index:
                                print("--------------")
                                print(f"The gps coordinates are: ({self.gps_coords[0]}, {self.gps_coords[1]})")
                                print(f"!!!! Motion Planning will be performed using: {segment_index} !!!!")

                if segment_index and segmented_planning[segment_index]["compute_algo"] and not segmented_planning[segment_index]["algo_done"]:
                    if not current_steering_input:
                        self.set_speed(0)
                        current_steering_input = self.compute_hybrid_a_star_path(segmented_planning[segment_index]["maze"])
                    else:
                        # print(f"entered else where local i is {local_i} and cehck is on {len(current_steering_input)}")
                        if local_i//100 < len(current_steering_input):
                            # print(f"Current speed: {segmented_planning[segment_index]['speed']}")
                            self.set_speed(segmented_planning[segment_index]["speed"])
                            self.set_steering_angle(math.radians(-1 * current_steering_input[local_i//100]))
                        else:
                            # TODO: Determine how to continue the previous algorithm
                            print("--------------")
                            print(f"The gps coordinates are: ({self.gps_coords[0]}, {self.gps_coords[1]})")
                            print(f"!!!! We are switching back to {prev_segment_index} planner from {segment_index} !!!!")
                            segmented_planning[segment_index]["algo_done"] = True
                            current_steering_input = []
                            if prev_segment_index:
                                segment_index = prev_segment_index

                if segment_index and not segmented_planning[segment_index]["compute_algo"]:
                    self.set_speed(segmented_planning[segment_index]["speed"])

                    if "deviation_check_index" in segmented_planning[segment_index]:
                        # print(f"Performing deviation correction as per {segment_index} planner")
                        compass_value = self.compass.getValues()
                        current_deviation = compass_value[segmented_planning[segment_index]["deviation_check_index"]]
                        if abs(current_deviation) >= 0.1:
                            self.set_steering_angle(current_deviation)
                            # print(f"Performing deviation correction as per {segment_index} planner by {current_deviation} radians")
                        else:
                            self.set_steering_angle(math.radians(segmented_planning[segment_index]["steering_angle"]))
                            # print(f"Performing deviation correction as per {segment_index} planner by {math.radians(segmented_planning[segment_index]['steering_angle'])} radians")
                    else:
                        self.set_steering_angle(math.radians(segmented_planning[segment_index]["steering_angle"]))

                if segment_index and "termination" in segment_index:
                    print("!!!! MOTION PLANNING COMPLETED !!!!")
                    print('------------------------------------------------------------------------------------------------')
                    break

                if self.config["sensors"]["display"]["enabled"]:
                    self.update_display()

            i += 1
            local_i += 1

        return 0  # ignored
    
    def save_gps_data(self):
        filename = "/Users/arjunramesh/aramesh/fun/webots_backup/controllers/av_driver/test2.csv"
        df = pl.DataFrame(self.gps_stream, schema=['x', 'y', 'z'])
        
        # Save the DataFrame to CSV if it has data
        if not df.is_empty():
            df.write_csv(filename)

# supervisor = Supervisor()
# driver_node = supervisor.getFromDef("vehicle")
avd = AVDriver()
avd.configure_sensors()
avd.main_loop()