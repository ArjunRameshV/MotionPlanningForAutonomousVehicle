class SensorsMixin:

    def configure_sick_lms(self):
        if self.config["sensors"]["sick_lms"]["enabled"]:
            print("Sick LMS is not currently supported")
            self.lms = None
        else:
            print("Sick LMS is disabled in the config")
            self.lms = None

    def configure_display(self):
        if self.config["sensors"]["display"]["enabled"]:
            # initialize display (speedometer)
            self.display = self.getDevice("display")
            self.speedometer_image = self.display.imageLoad("speedometer.png")
        else:
            print("Display is disabled in the config")
            self.display = None

    def configure_camera(self):
        # camera device
        if self.config["sensors"]["camera"]["enabled"]:
            self.camera = self.getDevice("camera")
            self.camera.enable(self.config["sensors"]["camera"].get("sampling_period", self.default_sampling_period))
        else:
            print("Camera is disabled in the config")
            self.camera = None

    def configure_gps(self):
        if self.config["sensors"]["gps"]["enabled"]:
            self.gps = self.getDevice("gps")
            self.gps.enable(self.config["sensors"]["gps"].get("sampling_period", self.default_sampling_period))
        else:
            print("GPS is disabled in the config")
            self.gps = None

    def configure_gyro(self):
        if self.config["sensors"]["gyro"]["enabled"]:
            self.gyro = self.getDevice("gyro")
            self.gyro.enable(self.config["sensors"]["gyro"].get("sampling_period", self.default_sampling_period))
        else:
            print("Gyro is disabled in the config")
            self.gyro = None

    def configure_compass(self):
        if self.config["sensors"]["compass"]["enabled"]:
            self.compass = self.getDevice("compass")
            self.compass.enable(self.config["sensors"]["compass"].get("sampling_period", self.default_sampling_period))
        else:
            print("Compass is disabled in the config")
            self.compass = None

    def configure_sensors(self):
        self.default_sampling_period = self.config["TIME_STEP"]
        
        for sensor_index in range(self.getNumberOfDevices()):
            device = self.getDeviceByIndex(sensor_index)
            name = device.getName()

            match name:
                case "Sick LMS 291":
                    self.configure_sick_lms()
                case "display":
                    self.configure_display()
                case "gps":
                    self.configure_gps()
                case "camera":
                    self.configure_camera()
                case "gyro":
                    self.configure_gyro()
                case "compass":
                    self.configure_compass()
