# config_manager.py
from configurations.default import CONFIG

class ConfigManager(dict):
    def __init__(self, initial_data=None):
        if initial_data is None:
            initial_data = CONFIG
        super().__init__(initial_data)

    def get_config(self, key):
        return self.config.get(key)

    def set_config(self, key, value):
        self.config[key] = value

    def update_configs(self, new_configs):
        self.config.update(new_configs)

'''
# Usage example
class Robot:
    def __init__(self, config):
        self.config = config

# Example usage
if __name__ == "__main__":
    manager = ConfigManager()
    robot = Robot(manager)
    
    # Accessing configuration values using dict-like access
    print(robot.config["TIME_STEP"])
    print(robot.config["sensors"]["gps"]["gps_coords"])
    print(robot.config["enable_display"])

    # Accessing configuration values using get_config method
    print(robot.config.get_config("TIME_STEP"))

    # Setting configuration values
    robot.config.set_config("TIME_STEP", 60)
    print(robot.config["TIME_STEP"])

    # Updating multiple configuration values
    new_configs = {
        "KP": 0.3,
        "KD": 2.5,
        "enable_collision_avoidance": True
    }
    robot.config.update_configs(new_configs)
    print(robot.config["KP"])
    print(robot.config["KD"])
    print(robot.config["enable_collision_avoidance"])
'''