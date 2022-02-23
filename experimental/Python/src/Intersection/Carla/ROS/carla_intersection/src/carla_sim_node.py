# ROS2 libraries
import rclpy
from rclpy.node import Node

# ROS2 messages
from carla_intersection_msgs.msg import VehicleCommand
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

# Other libraries
import glob
import os
import sys
try:
    import queue
except ImportError:
    import Queue as queue
from src.utils import make_coordinate, make_spawn_point

# Constants
from src.constants import CARLA_INSTALL_DIR

# Set up Carla
try:
    sys.path.append(glob.glob(CARLA_INSTALL_DIR + "/PythonAPI/carla/dist/carla-*%d.%d-%s.egg" % (
        sys.version_info.major,
        sys.version_info.minor,
        "win-amd64" if os.name == "nt" else "linux-x86_64"))[0])
except IndexError:
    pass
try:
    import carla
except ImportError:
    sys.stderr.write("ERROR: Could not find the Carla .egg file.\nPlease make sure that"
    " CARLA_INSTALL_DIR in CarlaIntersection.lf points to the correct location.\n")

# The Carla Simulator
class CarlaSim(Node):
    def __init__(self):
        super().__init__("carla_sim")

        # Parameters declaration
        self.declare_parameter('interval', 16) # msec
        self.declare_parameter('vehicle_id', 0)
        self.declare_parameter('initial_speed', [0.0, 0.0, 0.0])
        self.declare_parameter('vehicle_type', 'vehicle.tesla.model3')
        self.declare_parameter('spawn_point', [0.0, 0.0, 0.0, 0.0])

        # State variables initialization
        self.interval = self.get_parameter('interval').value
        self.vehicle_id = self.get_parameter('vehicle_id').value
        self.vehicle_type = self.get_parameter('vehicle_type').value
        self.initial_speed = make_coordinate(self.get_parameter('initial_speed').value)
        self.spawn_point = make_spawn_point(self.get_parameter('spawn_point').value)
        self.world_is_ready = False

        # pubsub for input and output ports
        self.status_ = self.create_publisher(Vector3, "status_to_vehicle_stats", 10)
        self.position_ = self.create_publisher(Vector3, "position_to_vehicle_pos", 10)
        self.command_ = self.create_subscription(VehicleCommand, "control_to_command", self.command_callback, 10)
        if self.vehicle_id == 0:
            self.world_is_ready_ = self.create_publisher(Bool, "world_is_ready", 10)
        else:
            self.world_is_ready_ = self.create_subscription(Bool, "world_is_ready", self.world_is_ready_callback, 10)

        self.initialize_carla()

        # timer (should be after initialize_carla() is called)
        self.timer_ = self.create_timer(self.interval / 1000.0, self.timer_callback)

    def command_callback(self, command):
        if command.vehicle_id != self.vehicle_id:
            return
        self.vehicle.apply_control( \
            carla.VehicleControl( \
                throttle=command.throttle, \
                brake=command.brake \
            ) \
        )

    def timer_callback(self):
        if not self.world_is_ready:
            return
        self.world.tick()
        velocity = self.vehicle.get_velocity()
        status = Vector3(x=velocity.x, y=velocity.y, z=velocity.z)
        self.status_.publish(status)
        
        gps_pos = self.gps_queue.get()
        position = Vector3(x = gps_pos.latitude, y = gps_pos.longitude, z= gps_pos.altitude)
        self.position_.publish(position)

    def world_is_ready_callback(self, _):
        self.world = self.client.get_world()
        self.world_is_ready = True
        self.initialize_vehicle(self.world)
        
    def initialize_carla(self):
        # initialize Carla
        self.client=carla.Client("localhost", 2000)
        self.client.set_timeout(10.0) # seconds
        if self.vehicle_id == 0:
            self.world = self.client.load_world("Town05")
            self.initialize_world(self.world)
            self.world_is_ready = True
            self.world_is_ready_.publish(Bool())
            self.initialize_vehicle(self.world)
        
    def initialize_world(self, world):
        settings = world.get_settings()
        settings.fixed_delta_seconds =  self.interval / 1000.0
        settings.substepping = True
        settings.max_substep_delta_time = settings.fixed_delta_seconds / 10
        settings.max_substeps = 10
        settings.synchronous_mode = True # Enables synchronous mode
        world.apply_settings(settings)
        self.set_weather()
        self.set_spectator_camera()
        
    def set_weather(self):
        # Set the weather
        weather = carla.WeatherParameters(
            cloudiness=80.0,
            precipitation=30.0,
            sun_altitude_angle=70.0)
        self.world.set_weather(weather)

    def set_spectator_camera(self):
        # Set the spectator (camera) position
        transform = carla.Transform(carla.Location(x=-126.163864, y=3, z=67), \
            carla.Rotation(pitch=-90, yaw=-180, roll=0))
        self.world.get_spectator().set_transform(transform)

    def initialize_vehicle(self, world):
        blueprint_library = world.get_blueprint_library()        
        
        sensors_bp = {}
        sensors = {}
        sensors_to_spawn = { \
            "gps": "sensor.other.gnss", \
            "imu": "sensor.other.imu"
        }
        
        spawn_point = self.spawn_point
        # Spawn the vehicle        
        vehicle_bp = blueprint_library.find(self.vehicle_type)
        transform = carla.Transform(carla.Location( \
                x = spawn_point.x, \
                y = spawn_point.y, \
                z = spawn_point.z \
                ), carla.Rotation(yaw = spawn_point.yaw))
        self.vehicle = self.world.spawn_actor(vehicle_bp, transform)
        
        for key in sensors_to_spawn.keys():
            sensors_bp[key] =  blueprint_library.find(sensors_to_spawn[key])
            if key == "gps":
                # Spawn the GPS sensor that is attached to the vehicle        
                relative_transform = carla.Transform(carla.Location( \
                        x = 1.0, \
                        y = 0.0, \
                        z = 2.0), carla.Rotation())
            
            elif key == "imu":
                # Spawn the imu unit
                relative_transform = carla.Transform(carla.Location( \
                        x = 2.0, \
                        y = 0.0, \
                        z = 2.0), carla.Rotation())
            else:
                relative_transform = carla.Transform(carla.Location(), carla.Rotation())
            
            sensors[key] = self.world.spawn_actor( \
                sensors_bp[key], \
                relative_transform, \
                attach_to=self.vehicle, \
                attachment_type=carla.AttachmentType.Rigid \
            )
        
        self.gps = sensors["gps"]
        self.gps_queue = queue.Queue()
        self.gps.listen(self.gps_queue.put)
        
        # Set the initial speed
        target_speed = self.initial_speed
        for i in range(1):
            self.vehicle.set_target_velocity(carla.Vector3D(x=target_speed.x, y=target_speed.y, z=target_speed.z))
            # self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
            # self.world.tick()
        
        self.get_logger().info("Spawned vehicle")


def main(args=None):
    rclpy.init(args=args)

    ego_vehicle = CarlaSim()

    rclpy.spin(ego_vehicle)

    # Destroy the node explicitly
    ego_vehicle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
