import numpy as np


def gps_to_cartesian(gps_coords):
    lat, lon = np.deg2rad(gps_coords[0]), np.deg2rad(gps_coords[1])
    R = 6371 # radius of the earth
    x = R * np.cos(lat) * np.cos(lon)
    y = R * np.cos(lat) * np.sin(lon)
    z = R *np.sin(lat)
    return (x,y,z)

def update_gps_readings(gps_sensor):
    coords = gps_sensor.getValues()
    speed_ms = gps_sensor.getSpeed()
    
    gps_speed = speed_ms * 3.6  # convert from m/s to km/h
    gps_coords = coords[:]

    return (gps_speed, gps_coords)