# ROS2 libraries
import rclpy
from rclpy.node import Node

# ROS2 messages
from carla_intersection_msgs.msg import Request, Grant, VehicleCommand
from geometry_msgs.msg import Vector3

# Other libraries
from math import sqrt
from src.utils import distance, make_coordinate, make_Vector3

# Constants
from src.constants import BILLION, SPEED_LIMIT, GOAL_REACHED_THRESHOLD, GOAL_REACHED_THRESHOLD_TIME

class VehicleNode(Node):
    def __init__(self):
        super().__init__(f"vehicle")

        # Parameters declaration
        self.declare_parameter('vehicle_id', 0)
        self.declare_parameter('initial_position', [0.0, 0.0, 0.0])

        # State variables initialization
        self.vehicle_id = self.get_parameter('vehicle_id').value
        self.initial_position = self.current_pos = make_coordinate(self.get_parameter('initial_position').value)
        self.granted_time_to_enter = 0
        self.intersection_position = None
        self.goal_reached = False
        self.velocity = 0.0
        self.asking_for_grant = False
        
        # pubsub for input output ports
        self.vehicle_stat_ = self.create_subscription(Vector3, "status_to_vehicle_stats", self.vehicle_stat_callback, 10)
        self.vehicle_pos_ = self.create_subscription(Vector3, "position_to_vehicle_pos", self.vehicle_pos_callback, 10)
        self.control_ = self.create_publisher(VehicleCommand, "control_to_command", 10)
        self.grant_ = self.create_subscription(Grant, "grant", self.grant_callback, 10)
        self.request_ = self.create_publisher(Request, "request", 10)

    def vehicle_pos_callback(self, vehicle_pos):
        self.current_pos = make_coordinate([vehicle_pos.x, vehicle_pos.y, vehicle_pos.z])

    def update_velocity(self, vehicle_stat):
        velocity_3d = make_coordinate([vehicle_stat.x, vehicle_stat.y, vehicle_stat.z])
        linear_speed = sqrt(velocity_3d.x**2 + velocity_3d.y**2 + velocity_3d.z**2)
        self.velocity = linear_speed
        if self.velocity == 0.0:
            # Prevent divisions by zero
            self.velocity = 0.001

    def vehicle_stat_callback(self, vehicle_stat):
        if self.goal_reached:
            return
        # Record the speed
        self.update_velocity(vehicle_stat)
        
        # Check if we have received an initial pos
        if distance(self.current_pos, make_coordinate([0, 0, 0])) <= 0.00000001:
            self.get_logger().info("Warning: Have not received initial pos yet.")
            return
        
        # Send a new request to the RSU if no time to enter
        # the intersection is granted
        if self.granted_time_to_enter == 0:
            if not self.asking_for_grant:
                request = Request()
                request.requestor_id = self.vehicle_id
                request.speed = self.velocity
                request.position = make_Vector3(self.current_pos)
                self.request_.publish(request)
                self.asking_for_grant = True

            # Stop the vehicle
            cmd = VehicleCommand()
            cmd.throttle = 0.0
            cmd.brake = 1.0
            self.control_.publish(cmd)
        else:
            # We have a granted time from the RSU
            # All we need to do is adjust our velocity
            # to enter the intersection at the allocated
            # time
            
            # First, how far are we from the intersection
            distance_remaining = distance(self.intersection_position, self.current_pos)
            current_time = self.get_clock().now().to_msg()
            time_remaining = (self.granted_time_to_enter - current_time.sec * BILLION - current_time.nanosec) / BILLION
            
            self.get_logger().info("########################################")
            self.get_logger().info("Vehicle {}: Distance to intersection: {}m.".format(self.vehicle_id + 1, distance_remaining))
            self.get_logger().info("Vehicle {}: Time to intersection: {}s.".format(self.vehicle_id + 1, time_remaining))
            self.get_logger().info("Vehicle {}: Current speed: {}m/s.".format(self.vehicle_id + 1, self.velocity))

            target_speed = 0.0
            # target_speed = distance_remaining/time_remaining
            
            if distance_remaining <= GOAL_REACHED_THRESHOLD and \
                    time_remaining <= GOAL_REACHED_THRESHOLD_TIME :
                # Goal reached
                # At this point, a normal controller should stop the vehicle until
                # it receives a new goal. However, for the purposes of this demo,
                # it will set the target speed to the speed limit so that vehicles
                # can leave the intersection (otherwise, they will just stop at the
                # intersection).
                target_speed = SPEED_LIMIT
                # Simulation is over
                self.goal_reached = True
                
                self.get_logger().info("\n\n*************************************************************\n\n".format(self.vehicle_id + 1))
                self.get_logger().info("************* Vehicle {}: Reached intersection! *************".format(self.vehicle_id + 1))
                self.get_logger().info("\n\n*************************************************************\n\n".format(self.vehicle_id + 1))

            elif time_remaining < (distance_remaining / SPEED_LIMIT):
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
            
            self.get_logger().info("Vehicle {}: Calculated target speed: {}m/s.".format(self.vehicle_id + 1, target_speed))
            
            if (target_speed - SPEED_LIMIT) > 0:
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
                throttle = min((target_speed - self.velocity)/target_speed, 1.0)
                # throttle = 1.0
                brake = 0.0
                # throttle = min(abs(target_speed / self.velocity), 1)
            else:
                # Need to apply the brake
                brake = min((self.velocity - target_speed)/self.velocity, 1.0)
                # brake = 1.0
                throttle = 0.0
            
            # Check throttle boundaries
            if throttle < 0:
                self.get_logger().info("Error: negative throttle")
                throttle = 0.0
            
            # Prepare and send the target velocity as a vehicle command
            cmd = VehicleCommand()
            cmd.vehicle_id = self.vehicle_id
            cmd.throttle = throttle
            cmd.brake = brake
            self.control_.publish(cmd)
            self.get_logger().info("Vehicle {}: Throttle: {}. Brake: {}".format(self.vehicle_id + 1, throttle, brake))


    def grant_callback(self, grant):
        if grant.requestor_id != self.vehicle_id:
            return
        self.get_logger().info("Vehicle {} Granted access".format(self.vehicle_id + 1) + 
            "to enter the intersection at elapsed logical time {:d}.\n".format(
                int(grant.arrival_time)
            )
        )
        self.granted_time_to_enter = grant.arrival_time
        self.intersection_position = grant.intersection_position
        self.asking_for_grant = False


def main(args=None):
    rclpy.init(args=args)

    ego_vehicle = VehicleNode()

    rclpy.spin(ego_vehicle)

    # Destroy the node explicitly
    ego_vehicle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()