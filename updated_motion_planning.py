# Import necessary modules
import math
import sys


from controller import Camera, Display, GPS, Keyboard, Lidar, Robot, Supervisor
from vehicle import Driver, Car

from enum import Enum

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import numpy as np
import threading
import polars as pl

import maze_env
from updated_search import HybridAStarSearch
from utils import convert_to_grid_coordinates

from utilities.env import generate_grid_world, visualize_grid_world

class Coordinate(Enum):
    X = 0
    Y = 1
    Z = 2

class AVDriver:
    def __init__(self):
        # wbu_driver_init()
        self.driver_init = Driver()
        self.keyboard = Keyboard()
        self.init_constants()
    
    # ----------------- CONSTANTS ----------------- #
    def init_constants(self):
        self.TIME_STEP = 50
        self.UNKNOWN = 99999.99

        # Line following PID
        self.KP = 0.25
        self.KI = 0.006
        self.KD = 2

        self.PID_need_reset = True

        # Size of the yellow line angle filter
        self.FILTER_SIZE = 3

        # enabe various 'features'
        self.enable_collision_avoidance = False
        self.enable_display = False
        self.has_gps = False
        self.has_camera = False
        self.has_gyro = False
        self.has_compass = False

        # camera
        # WbDeviceTag camera;
        self.camera_width = -1
        self.camera_height = -1
        self.camera_fov = -1.0

        # SICK laser
        # WbDeviceTag sick;
        self.sick_width = -1
        self.sick_range = -1.0
        self.sick_fov = -1.0

        # speedometer
        # WbDeviceTag display;
        self.display_width = 0
        self.display_height = 0
        # WbImageRef speedometer_image = NULL;

        # GPS
        # WbDeviceTag gps;
        self.gps_coords = [0.0, 0.0, 0.0]
        self.gps_speed = 0.0

        # misc variables
        self.speed = 0.0
        self.steering_angle = 0.0
        self.manual_steering = 0
        self.autodrive = True
        self.obstacle_dist = 0.0
        self.write_lidar = False

    # ----------------- HELPER FUNCTIONS ----------------- #

    # set target speed
    def set_speed(self, kmh):
        # max speed
        # if kmh > 50.0:
        #     kmh = 50.0

        self.speed = kmh

        # print(f"setting speed to {kmh} km/h")
        self.driver.setCruisingSpeed(kmh)

    def print_help(self):
        print("You can drive this car!")
        print("Select the 3D window and then use the cursor keys to:")
        print("[LEFT]/[RIGHT] - steer")
        print("[UP]/[DOWN] - accelerate/slow down")
        
    # returns approximate angle of yellow road line
    # or UNKNOWN if no pixel of yellow line visible
    def process_camera_image(self, image):
        num_pixels = self.camera_height * self.camera_width  # number of pixels in the image
        REF = np.array([95, 187, 203])  # road yellow (BGR format)
        # REF = np.array([255, 255, 255])  # road yellow (BGR format)
        sumx = 0  # summed x position of pixels
        pixel_count = 0  # yellow pixels count

        pixel = image
        for x in range(num_pixels):
            if self.color_diff(pixel, REF) < 30:
                sumx += x % self.camera_width
                pixel_count += 1  # count yellow pixels
            pixel = pixel[4:] # move the pixel to the next four memory space

        # if no pixels were detected...
        if pixel_count == 0:
            return self.UNKNOWN

        return ((sumx / pixel_count / self.camera_width) - 0.5) * self.camera_fov
    
    def color_diff(self, a, b):
        return sum(abs(a[i] - b[i]) for i in range(3))

    def process_sick_data(self, sick_data):
        HALF_AREA = 20  # check 20 degrees wide middle area
        sumx = 0
        collision_count = 0
        self.obstacle_dist = 0.0

        for x in range(self.sick_width // 2 - HALF_AREA, self.sick_width // 2 + HALF_AREA):
            range_val = sick_data[x]
            if range_val < 20.0:
                sumx += x
                collision_count += 1
                self.obstacle_dist += range_val

        # if no obstacle was detected...
        if collision_count == 0:
            return self.UNKNOWN

        self.obstacle_dist /= collision_count
        return ((sumx / collision_count / self.sick_width) - 0.5) * self.sick_fov
    
    def compute_gps_speed(self):
        coords = self.gps.getValues()
        speed_ms = self.gps.getSpeed()
        # store into global variables
        self.gps_speed = speed_ms * 3.6  # convert from m/s to km/h
        self.gps_coords = coords[:]

    # positive: turn right, negative: turn left
    def set_steering_angle(self, wheel_angle):
        # limit the difference with previous steering_angle
        # if wheel_angle - self.steering_angle > 0.1:
        #     wheel_angle = self.steering_angle + 0.1
        # if wheel_angle - self.steering_angle < -0.1:
        #     wheel_angle = self.steering_angle - 0.1
        
        new_angle = self.steering_angle + wheel_angle
        
        # limit range of the steering angle
        if new_angle > 0.5:
            new_angle = 0.5
        elif new_angle < -0.5:
            new_angle = -0.5
        
        self.driver.setSteeringAngle(new_angle)

    def change_manual_steer_angle(self, inc):
        # self.set_autodrive(False)

        if inc == 0:
            self.manual_steering = 0
            self.set_steering_angle(0)

        new_manual_steering = self.manual_steering + inc
        if -25.0 <= new_manual_steering <= 25.0:
            self.manual_steering = new_manual_steering
            self.set_steering_angle(self.manual_steering * 0.02)

        if self.manual_steering == 0:
            pass
        else:
            pass

    def check_keyboard(self):
        key = self.keyboard.getKey()
        if key == self.keyboard.UP:
            self.set_speed(self.speed + 1.0)
        elif key == self.keyboard.DOWN:
            self.set_speed(self.speed - 1.0)
        elif key == self.keyboard.RIGHT:
            self.change_manual_steer_angle(+1)
        elif key == self.keyboard.LEFT:
            self.change_manual_steer_angle(-1)
        elif key == 'A':
            self.set_autodrive(True)
        elif not self.autodrive:
            self.change_manual_steer_angle(0)

    def set_autodrive(self, onoff):
        if self.autodrive == onoff:
            return
        self.autodrive = onoff
        if not self.autodrive:
            pass
        else:
            if self.has_camera:
                pass
            else:
                pass

    # ----------------- DISPLAY FUNCTION ----------------- #
                
    def update_display(self):
        NEEDLE_LENGTH = 50.0

        # display background
        self.display.imagePaste(self.speedometer_image, 0, 0, False)

        # draw speedometer needle
        current_speed = self.driver.getCurrentSpeed()
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
    def gps_to_cartesian(self):
        lat, lon = np.deg2rad(self.gps_coords[0]), np.deg2rad(self.gps_coords[1])
        R = 6371 # radius of the earth
        x = R * np.cos(lat) * np.cos(lon)
        y = R * np.cos(lat) * np.sin(lon)
        z = R *np.sin(lat)
        return (x,y,z)

    def main_setup(self):
        self.driver = self.driver_init
        
        # check if there is a SICK and a display
        for j in range(self.driver.getNumberOfDevices()):
            device = self.driver.getDeviceByIndex(j)
            name = device.getName()
            # name = self.driver.getName(device)
            if name == "Sick LMS 291":
                self.enable_collision_avoidance = True
            elif name == "display":
                self.enable_display = True
            elif name == "gps":
                self.has_gps = True
            elif name == "camera":
                self.has_camera = True
            elif name == "lidar":
                self.has_lidar = True
            elif name == "gyro":
                self.has_gyro = True
            elif name == "compass":
                self.has_compass = True
            
            # self.enable_collision_avoidance = False

        # camera device
        if self.has_camera:
            self.camera = self.driver.getDevice("camera")
            self.camera.enable(self.TIME_STEP)
            self.camera_width = self.camera.getWidth()
            self.camera_height = self.camera.getHeight()
            self.camera_fov = self.camera.getFov()

        if self.has_gyro:
            self.gyro = self.driver.getDevice("gyro")
            self.gyro.enable(self.TIME_STEP)
        
        if self.has_compass:
            self.compass = self.driver.getDevice("compass")
            self.compass.enable(self.TIME_STEP)

        # initialize gps
        if self.has_gps:
            self.gps = self.driver.getDevice("gps")
            self.gps.enable(self.TIME_STEP)

        # initialize display (speedometer)
        if self.enable_display:
            self.display = self.driver.getDevice("display")
            self.speedometer_image = self.display.imageLoad("speedometer.png")

        # start engine
        if self.has_camera:
            self.set_speed(5.0)  # km/h

        # set driver configs
        self.driver.setHazardFlashers(True)
        self.driver.setDippedBeams(True)
        self.driver.setAntifogLights(True)
        self.driver.setWiperMode(Driver.SLOW)

        # self.visualizer.start_visualization()

        # allow to switch to manual control
        self.keyboard.enable(self.TIME_STEP)

        self.write_gps = False

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
        while self.driver.step() != -1:
            # get user input
            self.check_keyboard()
            # updates sensors only every TIME_STEP milliseconds
            self.TIME_STEP = 1000
            if i % (int(self.TIME_STEP / self.driver.getBasicTimeStep())) == 0:
                if self.has_camera:
                    # capture the front camera image
                    self.camera.getImage()

                # update sensor data
                if self.has_gps:
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
                            print(f"Performing deviation correction as per {segment_index} planner by {current_deviation} radians")
                        else:
                            self.set_steering_angle(math.radians(segmented_planning[segment_index]["steering_angle"]))
                            print(f"Performing deviation correction as per {segment_index} planner by {math.radians(segmented_planning[segment_index]['steering_angle'])} radians")
                    else:
                        self.set_steering_angle(math.radians(segmented_planning[segment_index]["steering_angle"]))

                if segment_index and "termination" in segment_index:
                    print("!!!! MOTION PLANNING COMPLETED !!!!")
                    print('------------------------------------------------------------------------------------------------')
                    break

                if self.enable_display:
                    self.update_display()

            i += 1
            local_i += 1

        self.driver = None
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
avd.main_setup()
avd.main_loop()