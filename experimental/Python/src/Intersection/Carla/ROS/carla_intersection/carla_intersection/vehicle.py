import rclpy
from rclpy.node import Node

import random

from std_msgs.msg import String

from carla_intersection_msgs.msg import Request, Grant
from geometry_msgs.msg import Vector3

class Vehicle(Node):
    def __init__(self):
        super().__init__(f"vehicle_{random.randint(0,1000)}")

        self.declare_parameter('vehicle_id', 0)
        
        self.vehicle_stat_ = self.create_publisher(Grant, "vehicle_grant", 10)
        self.request_ = self.create_subscription(Request, "rsu_request", self.request_callback, 10)



def main(args=None):
    rclpy.init(args=args)

    ego_vehicle = Vehicle()

    rclpy.spin(ego_vehicle)

    # Destroy the node explicitly
    ego_vehicle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()