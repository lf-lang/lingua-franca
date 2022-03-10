# ROS2 libraries
import rclpy
from rclpy.node import Node

# ROS2 messages
from carla_intersection_msgs.msg import Request, Grant, VehicleCommand
from geometry_msgs.msg import Vector3

# Other libraries
from src.utils import make_coordinate, ROSClock
from src.ros_utils import make_Vector3
from src.vehicle import Vehicle

# Constants
from src.constants import BILLION

class VehicleNode(Node):
    def __init__(self):
        super().__init__(f"vehicle")

        # Parameters declaration
        self.declare_parameter('vehicle_id', 0)
        self.declare_parameter('initial_velocity', [0.0, 0.0, 0.0])
        self.declare_parameter('initial_position', [0.0, 0.0, 0.0])

        # State variables initialization
        self.vehicle_id = self.get_parameter('vehicle_id').value
        self.initial_position = make_coordinate(self.get_parameter('initial_position').value)
        self.initial_velocity = make_coordinate(self.get_parameter('initial_velocity').value)
        self.asking_for_grant = False
        self.vehicle = Vehicle(vehicle_id = self.vehicle_id, 
                               initial_position = self.initial_position, 
                               initial_velocity = self.initial_velocity,
                               clock = ROSClock(self.get_clock()),
                               logger = self.get_logger())
        
        # pubsub for input output ports
        self.velocity_ = self.create_subscription(Vector3, f"vehicle_velocity_{self.vehicle_id}", self.velocity_callback, 10)
        self.position_ = self.create_subscription(Vector3, f"vehicle_position_{self.vehicle_id}", self.position_callback, 10)
        self.control_ = self.create_publisher(VehicleCommand, f"control_to_command_{self.vehicle_id}", 10)
        self.grant_ = self.create_subscription(Grant, "grant", self.grant_callback, 10)
        self.request_ = self.create_publisher(Request, "request", 10)

    def position_callback(self, new_position):
        self.vehicle.set_position(new_position)

    def velocity_callback(self, new_velocity):
        pub_packets = self.vehicle.receive_velocity_from_simulator(new_velocity)
        if pub_packets.cmd != None:
            cmd = VehicleCommand()
            cmd.throttle = pub_packets.cmd.throttle
            cmd.brake = pub_packets.cmd.brake
            self.control_.publish(cmd)
        if pub_packets.request != None and not self.asking_for_grant:
            request = Request()
            request.requestor_id = self.vehicle_id
            request.speed = pub_packets.request.speed
            request.position = make_Vector3(pub_packets.request.position)
            self.request_.publish(request)
            self.asking_for_grant = True

    def grant_callback(self, grant):
        if grant.requestor_id != self.vehicle_id:
            return
        self.vehicle.grant(grant.arrival_time, grant.intersection_position)
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