import numpy as np


def gps_to_cartesian(gps_coords):
    lat, lon = np.deg2rad(gps_coords[0]), np.deg2rad(gps_coords[1])
    R = 6371 # radius of the earth
    x = R * np.cos(lat) * np.cos(lon)
    y = R * np.cos(lat) * np.sin(lon)
    z = R *np.sin(lat)
    return (x,y,z)