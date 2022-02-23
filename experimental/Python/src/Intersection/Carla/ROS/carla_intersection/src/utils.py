from math import sin, cos, sqrt, atan2, radians
from geometry_msgs.msg import Vector3
class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

def distance(coordinate1, coordinate2):        
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    Taken from: https://stackoverflow.com/a/15737218/783868
    """
    # Currently ignores altitude
    # Convert decimal degrees to radians 
    lat1 = radians(coordinate1.x)
    lon1 = radians(coordinate1.y)
    lat2 = radians(coordinate2.x)
    lon2 = radians(coordinate2.y)
    
    # Haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a)) 
    # Radius of earth in kilometers is 6371
    km = 6371.0 * c
    m = km * 1000.0
    return m

def make_coordinate(list):
    return dotdict({"x": list[0], "y": list[1], "z": list[2]})

def make_spawn_point(list):
    return dotdict({"x": list[0], "y": list[1], "z": list[2], "yaw": list[3]})

def make_Vector3(coordinate):
    return Vector3(x=coordinate.x, y=coordinate.y, z=coordinate.z)
