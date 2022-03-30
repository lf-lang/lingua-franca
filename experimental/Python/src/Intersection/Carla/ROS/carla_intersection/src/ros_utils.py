from geometry_msgs.msg import Vector3

def make_Vector3(coordinate):
    return Vector3(x=coordinate.x, y=coordinate.y, z=coordinate.z)