import rclpy
from rclpy.node import Node

import random
from math import sin, cos, sqrt, atan2, radians, pi

from std_msgs.msg import String

from carla_intersection_msgs.msg import Request, Grant, VehicleCommand
from geometry_msgs.msg import Vector3

BILLION = 1000000000

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


class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__


class Vehicle(Node):
    def __init__(self):
        super().__init__(f"vehicle_{random.randint(0,1000)}")
        self.vehicle_id = self.declare_parameter('vehicle_id', 0)
        self.intersection_pos = self.declare_parameter('intersection_pos', [0, 0, 0])

        self.granted_time_to_enter = 0
        self.intersection_pos = None
        self.goal_reached = False
        self.velocity = 0.0
        
        # pubsub for input output ports
        self.vehicle_stat_ = self.create_subscription(Vector3, "vehicle_stat", self.vehicle_stat_callback, 10)
        self.vehicle_pos_ = self.create_subscription(Vector3, "vehicle_pos", self.vehicle_pos_callback, 10)
        self.control_ = self.create_publisher(VehicleCommand, "control_to_command", 10)
        self.grant_ = self.create_subscription(Grant, "grant", self.grant_callback, 10)
        self.request_ = self.create_publisher(Request, "request", 10)
        
    def get_vehicle_id(self):
        return int(self.vehicle_id.value)

    def get_intersection_pos(self):
        p = self.intersection_pos.value
        return Vector3(x=p[0], y=p[1], z=p[2])

    def set_intersection_pos(self, intersection_pos):
        self.intersection_pos = dotdict({"x": intersection_pos.x, "y": intersection_pos.y, "z": intersection_pos.z})

    def vehicle_pos_callback(self, vehicle_pos):
        self.current_pos = Vector3(x=vehicle_pos.x, y=vehicle_pos.y, z=vehicle_pos.z)

    def update_velocity(self, vehicle_stat):
        velocity_3d = Vector3(x=vehicle_stat.x, y=vehicle_stat.y, z=vehicle_stat.z)
        linear_speed = sqrt(velocity_3d.x**2 + velocity_3d.y**2 + velocity_3d.z**2)
        self.velocity = linear_speed
        if self.velocity == 0:
            # Prevent divisions by zero
            self.velocity = 0.001


    def vehicle_stat_callback(self, vehicle_stat):
        if self.goal_reached:
            return
        # Record the speed
        self.update_velocity(vehicle_stat)
        
        # Check if we have received an initial pos
        if self.current_pos.distance(coordinate(0.0, 0.0, 0.0)) <= 0.00000001:
            self.get_logger().info("Warning: Have not received initial pos yet.")
            return
        
        # Send a new request to the RSU if no time to enter
        # the intersection is granted
        if self.granted_time_to_enter == 0:
            request = Request()
            request.requestor_id = self.get_vehicle_id()
            request.speed = self.velocity
            request.position = Vector3(x=self.current_pos.x, y=self.current_pos.y, z=self.current_pos.z)
            self.request_.publish(request)

            # Stop the vehicle
            cmd = VehicleCommand()
            cmd.throttle = 0
            cmd.brake = 1
            self.control_.publish(cmd)
        else:
            # We have a granted time from the RSU
            # All we need to do is adjust our velocity
            # to enter the intersection at the allocated
            # time
            
            # First, how far are we from the intersection
            distance_remaining = distance(self.get_intersection_pos(), self.current_pos)
            time_remaining = (self.granted_time_to_enter - self.get_clock().now().to_msg() * BILLION) / (BILLION * 1.0)
            
            self.get_logger().info("########################################")
            self.get_logger().info("Vehicle {}: Distance to intersection: {}m.".format(self.get_vehicle_id() + 1, distance_remaining))
            self.get_logger().info("Vehicle {}: Time to intersection: {}s.".format(self.get_vehicle_id() + 1, time_remaining))
            self.get_logger().info("Vehicle {}: Current speed: {}m/s.".format(self.get_vehicle_id() + 1, self.velocity))

            target_speed = 0.0
            # target_speed = distance_remaining/time_remaining
                        
            if distance_remaining <= goal_reached_threshold and \
                    time_remaining <= goal_reached_threshold_time :
                # Goal reached
                # At this point, a normal controller should stop the vehicle until
                # it receives a new goal. However, for the purposes of this demo,
                # it will set the target speed to the speed limit so that vehicles
                # can leave the intersection (otherwise, they will just stop at the
                # intersection).
                target_speed = speed_limit
                # Simulation is over
                self.goal_reached = True
                
                self.get_logger().info("\n\n*************************************************************\n\n".format(self.get_vehicle_id() + 1))
                self.get_logger().info("************* Vehicle {}: Reached intersection! *************".format(self.get_vehicle_id() + 1))
                self.get_logger().info("\n\n*************************************************************\n\n".format(self.get_vehicle_id() + 1))

                goal_reached.set(True)
            elif time_remaining < (distance_remaining / speed_limit):
                # No time to make it to the intersection even if we
                # were going at the speed limit.
                # Ask the RSU again
                self.granted_time_to_enter = 0
                # Apply the brake since we ran out of time
                target_speed = 0
            else:
                # Has not reached the goal
                # target_speed = ((2 * distance_remaining) / (time_remaining)) - self.velocity
                target_speed = distance_remaining / time_remaining
            
            self.get_logger().info("Vehicle {}: Calculated target speed: {}m/s.".format(self.get_vehicle_id() + 1, target_speed))
            
            if (target_speed - speed_limit) > 0:
                self.get_logger().info("Warning: target speed exceeds the speed limit")
                target_speed = 0
                self.granted_time_to_enter = 0
            
            if target_speed <= 0:
                self.get_logger().info("Warning: target speed negative or zero")
                target_speed = 0.001
                self.granted_time_to_enter = 0
            
            brake = 0.0
            throttle = 0.0
            
            if target_speed >= self.velocity:            
                # Calculate a proportional throttle (0.0 < throttle < 1.0)
                throttle = min((target_speed - self.velocity)/target_speed, 1)
                # throttle = 1.0
                brake = 0.0
                # throttle = min(abs(target_speed / self.velocity), 1)
            else:
                # Need to apply the brake
                brake = min((self.velocity - target_speed)/self.velocity, 1)
                # brake = 1.0
                throttle = 0.0
            
            # Check throttle boundaries
            if throttle < 0:
                self.get_logger().info("Error: negative throttle")
                throttle = 0
            
            # Prepare and send the target velocity as a vehicle command
            cmd = VehicleCommand()
            cmd.throttle = throttle
            cmd.brake = brake
            self.control_.publish(cmd)
            self.get_logger().info("Vehicle {}: Throttle: {}. Brake: {}".format(self.get_vehicle_id() + 1, throttle, brake))


    def grant_callback(self, grant):
        self.get_logger().info("Vehicle {} Granted access".format(self.vehicle_id + 1),
            "to enter the intersection at elapsed logical time {:d}.\n".format(
                int(grant.arrival_time)
            )
        )
        self.granted_time_to_enter = grant.arrival_time
        self.set_intersection_pos(grant.intersection_position)


def main(args=None):
    rclpy.init(args=args)

    ego_vehicle = Vehicle()

    rclpy.spin(ego_vehicle)

    # Destroy the node explicitly
    ego_vehicle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()