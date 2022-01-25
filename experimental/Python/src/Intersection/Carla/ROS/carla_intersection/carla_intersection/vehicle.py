import rclpy
from rclpy.node import Node

import random
from math import sin, cos, sqrt, atan2, radians, pi

from std_msgs.msg import String

from carla_intersection_msgs.msg import Request, Grant, VehicleCommand
from geometry_msgs.msg import Vector3

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


class Vehicle(Node):
    def __init__(self):
        super().__init__(f"vehicle_{random.randint(0,1000)}")
        self.declare_parameter('vehicle_id', 0)
        self.declare_parameter('intersection_pos', 0)

        self.granted_time_to_enter = 0
        self.intersection_pos = None
        self.goal_reached = False
        
        # velocity of vehicle as (vx, vy, vz)
        self.vehicle_stat_ = self.create_subscription(Vector3, "vehicle_stat", self.vehicle_stat_callback, 10)

        # position of vehicle as (x, y, z)
        self.vehicle_pos_ = self.create_subscription(Vector3, "vehicle_pos", self.vehicle_pos_callback, 10)

        self.control_ = self.create_publisher(VehicleCommand, "control_to_command", 10)

        # request and grants
        self.grant_ = self.create_subscription(Grant, "grant", self.grant_callback, 10)
        self.request_ = self.create_publisher(Request, "request", 10)
        
    def vehicle_pos_callback(self, vehicle_pos):
        self.current_pos = vehicle_pos

    def vehicle_stat_callback(self, vehicle_stat):
        if self.goal_reached:
            return
        # Record the speed
        velocity_3d = vehicle_stat
        linear_speed = sqrt(velocity_3d.x**2 + velocity_3d.y**2 + velocity_3d.z**2)
        self.velocity = linear_speed

        if self.velocity == 0:
            # Prevent divisions by zero
            self.velocity = 0.001
        
        # Check if we have received an initial pos
        if self.current_pos.distance(coordinate(0.0, 0.0, 0.0)) <= 0.00000001:
            print("Warning: Have not received initial pos yet.")
            return
        

    def grant_callback(self, grant):
        print("Vehicle {} Granted access".format(self.vehicle_id + 1),
            "to enter the intersection at elapsed logical time {:d}.\n".format(
                int(grant.arrival_time)
            )
        )
        self.granted_time_to_enter = grant.arrival_time
        self.intersection_pos = grant.intersection_pos


def main(args=None):
    rclpy.init(args=args)

    ego_vehicle = Vehicle()

    rclpy.spin(ego_vehicle)

    # Destroy the node explicitly
    ego_vehicle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()