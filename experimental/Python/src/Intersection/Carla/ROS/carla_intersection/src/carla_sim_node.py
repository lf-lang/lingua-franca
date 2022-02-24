# ROS2 libraries
import rclpy
from rclpy.node import Node

# ROS2 messages
from carla_intersection_msgs.msg import VehicleCommand
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

# Other libraries
from src.utils import make_coordinate, make_spawn_point, make_Vector3
from src.carla_sim import CarlaSim

# The Carla Simulator
class CarlaSimNode(Node):
    def __init__(self):
        super().__init__("carla_sim_node")

        # Parameters declaration
        self.declare_parameter('interval', 16) # msec
        self.declare_parameter('vehicle_id', 0)
        self.declare_parameter('initial_velocity', [0.0, 0.0, 0.0])
        self.declare_parameter('vehicle_type', 'vehicle.tesla.model3')
        self.declare_parameter('spawn_point', [0.0, 0.0, 0.0, 0.0])

        # State variables initialization
        self.interval = self.get_parameter('interval').value
        self.vehicle_id = self.get_parameter('vehicle_id').value
        self.vehicle_type = self.get_parameter('vehicle_type').value
        self.initial_velocity = make_coordinate(self.get_parameter('initial_velocity').value)
        self.spawn_point = make_spawn_point(self.get_parameter('spawn_point').value)
        self.world_is_ready = False
    
        # pubsub for input and output ports
        self.status_ = self.create_publisher(Vector3, f"status_to_vehicle_stats_{self.vehicle_id}", 10)
        self.position_ = self.create_publisher(Vector3, f"position_to_vehicle_pos_{self.vehicle_id}", 10)
        self.command_ = self.create_subscription(VehicleCommand, f"control_to_command_{self.vehicle_id}", self.command_callback, 10)
        
        self.carla_sim = CarlaSim(interval=self.interval, vehicle_type=self.vehicle_type, 
                                  initial_velocity=self.initial_velocity, spawn_point=self.spawn_point,
                                  logger=self.get_logger())
        self.carla_sim.initialize_carla()
        if self.vehicle_id == 0:
            self.world_is_ready_ = self.create_publisher(Bool, "world_is_ready", 10)
            self.carla_sim.initialize_world()
            self.world_is_ready_.publish(Bool())
            self.world_is_ready_callback()
        else:
            self.world_is_ready_ = self.create_subscription(Bool, "world_is_ready", self.world_is_ready_callback, 10)

        # timer (should be after initialize_carla() is called)
        self.timer_ = self.create_timer(self.interval / 1000.0, self.timer_callback)

    def command_callback(self, command):
        self.carla_sim.apply_control(command.throttle, command.brake)

    def timer_callback(self):
        if not self.world_is_ready:
            return
        self.carla_sim.tick()
        self.status_.publish(make_Vector3(self.carla_sim.get_vehicle_velocity()))
        position = self.carla_sim.get_vehicle_position()
        coordinate = make_coordinate([position.latitude, position.longitude, position.altitude])
        self.position_.publish(make_Vector3(coordinate))

    def world_is_ready_callback(self, _=None):
        self.carla_sim.get_world()
        self.world_is_ready = True
        self.carla_sim.initialize_vehicle()
        

def main(args=None):
    rclpy.init(args=args)

    ego_vehicle = CarlaSimNode()

    rclpy.spin(ego_vehicle)

    # Destroy the node explicitly
    ego_vehicle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
