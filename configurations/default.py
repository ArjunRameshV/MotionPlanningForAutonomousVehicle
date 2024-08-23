CONFIG = {
    # Basic Robot configs
    "TIME_STEP": 50,
    "UNKNOWN": 99999.99,

    # PID controller configs
    "pid": {
        "KP": 0.25,
        "KI": 0.006,
        "KD": 2,
        "PID_need_reset": True,
    },

    # Vehicle settings
    "vehicle_settings": {
        "enable_collision_avoidance": False,
        "autodrive_enabled": True,
        "initial_steering_angle": 0.0,
        "manual_steering": 0,
        "initial_speed": 0.0,
        "initial_coords": [0.0, 0.0, 0.0],
    },

    # Sensor settings
    "sensors": {
        "gps":  {
            "enabled": True,
        },
        "camera": {
            "enabled": True,
        },
        "gyro": {
            "enabled": True,
        },
        "compass": {
            "enabled": True,
        },
        "display": {
            "enabled": True,
        },
        "sick_lms": {
            "enabled": False,
        }
    },

    # Other settings
    "obstacle_dist": 0.0,
    "write_lidar": False
}
