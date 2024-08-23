# Import necessary modules
import math
import sys


from config_manager import ConfigManager
from controller import Camera, Display, GPS, Keyboard, Lidar, Robot, Supervisor
from gps_processing.utils import update_gps_readings
from manual_controller import ManualControllerMixin
from sample_planner import Planner
from sensors import SensorsMixin
from vehicle import Driver, Car

from enum import Enum

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import numpy as np
import polars as pl


class Coordinate(Enum):
    X = 0
    Y = 1
    Z = 2

class AVDriver(Car, ManualControllerMixin, SensorsMixin):
    def __init__(self):
        super().__init__()
        self.keyboard = Keyboard()
        self.config = ConfigManager()
        self.planner = Planner()
        # self.init_constants()

        # WbDeviceTag gps;
        self.gps_coords = self.config["vehicle_settings"]["initial_coords"]
        self.gps_speed = self.config["vehicle_settings"]["initial_speed"]

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

    def print_infomatics(self):
        print("You can drive this car!")
        print("Select the 3D window and then use the cursor keys to:")
        print("[LEFT]/[RIGHT] - steer")
        print("[UP]/[DOWN] - accelerate/slow down")

    def set_speed(self, kmph):
        # max speed
        if kmph > 50.0:
            kmph = 50.0

        self.speed = kmph
        self.setCruisingSpeed(kmph)

    # positive: turn right, negative: turn left
    def set_steering_angle(self, wheel_angle):
        # limit the difference with previous steering_angle
        if wheel_angle - self.steering_angle > 0.2:
            wheel_angle = self.steering_angle + 0.2
        if wheel_angle - self.steering_angle < -0.2:
            wheel_angle = self.steering_angle - 0.2
        
        new_angle = self.steering_angle + wheel_angle
        
        # limit range of the steering angle
        if new_angle > 0.5:
            new_angle = 0.5
        elif new_angle < -0.5:
            new_angle = -0.5
        
        self.setSteeringAngle(wheel_angle)

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

    def main_loop(self):
        # main loop
        i = 1
        self.gps_stream = []
        print('------------------------------------------------------------------------------------------------')
        while self.step() != -1:
            # get user input
            self.check_keyboard()
            # updates sensors only every self.config["TIME_STEP"] milliseconds
            time_step = 1000
            if i % (int(time_step / self.getBasicTimeStep())) == 0:
                if self.gps is None:
                    print("Waiting for GPS data")
                    continue
                
                # sensor readings
                gps_speed, gps_coords = update_gps_readings(self.gps)
                print(gps_coords)
                compass_value = self.compass.getValues()

                speed, steering_angle = self.planner.move(gps_coords, compass_value)
                print(speed, steering_angle)

                if speed == -1 and steering_angle == -1:
                    self.setSteeringAngle(0)
                    self.set_speed(0)
                    break

                if speed:
                    self.set_speed(speed)
                if steering_angle:
                    self.set_steering_angle(steering_angle)

                if self.config["sensors"]["display"]["enabled"]:
                    self.update_display()

            i += 1

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