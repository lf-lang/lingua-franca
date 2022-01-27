import rclpy
from rclpy.node import Node
from carla_intersection_msgs.msg import VehicleCommand
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
import time
import glob
import os
import sys
CARLA_INSTALL_DIR = "/opt/carla-simulator"
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

try:
    import queue
except ImportError:
    import Queue as queue

class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

# The Carla Simulator
class CarlaSim(Node):
    def __init__(self):
        super().__init__("carla_sim")
        self.interval = int(self.declare_parameter('interval', 16).value) # msec
        self.vehicle_id = self.declare_parameter('vehicle_id', 0)
        self.initial_speed = self.declare_parameter('initial_speed', [0.0, 0.0, 0.0])
        self.vehicle_type = self.declare_parameter('vehicle_type', 'vehicle.tesla.model3')
        self.spawn_point = self.declare_parameter('spawn_point', [0.0, 0.0, 0.0, 0.0])
        self.world_is_ready = False

        # pubsub for input and output ports
        self.status_ = self.create_publisher(Vector3, "status_to_vehicle_stats", 10)
        self.position_ = self.create_publisher(Vector3, "position_to_vehicle_pos", 10)
        self.command_ = self.create_subscription(VehicleCommand, "control_to_command", self.command_callback, 10)
        if self.get_vehicle_id() == 0:
            self.world_is_ready_ = self.create_publisher(Bool, "world_is_ready", 10)
        else:
            self.world_is_ready_ = self.create_subscription(Bool, "world_is_ready", self.world_is_ready_callback, 10)

        self.initialize_carla()

        # timer (should be after initialize_carla() is called)
        self.timer_ = self.create_timer(self.interval / 1000.0, self.timer_callback)


    def get_spawn_point(self):
        sp = self.spawn_point.value
        return dotdict({"x": sp[0], "y": sp[1], "z": sp[2], "yaw": sp[3]})
    
    def get_vehicle_type(self):
        return str(self.vehicle_type.value)

    def get_initial_speed(self):
        sp = self.initial_speed.value
        return Vector3(x=sp[0], y=sp[1], z=sp[2])

    def get_vehicle_id(self):
        return int(self.vehicle_id.value)

    def command_callback(self, command):
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
        if self.get_vehicle_id() == 0:
            self.world = self.client.load_world("Town05")
            self.initialize_world(self.world)
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
        
        spawn_point = self.get_spawn_point()
        # Spawn the vehicle        
        vehicle_bp = blueprint_library.find(self.get_vehicle_type())
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
        target_speed = self.get_initial_speed()
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
