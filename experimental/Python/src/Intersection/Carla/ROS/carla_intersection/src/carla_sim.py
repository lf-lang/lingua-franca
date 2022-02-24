# Other libraries
import glob
import os
import sys
try:
    import queue
except ImportError:
    import Queue as queue

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


class CarlaSim:
    def __init__(self, interval, vehicle_type, initial_velocity, spawn_point, logger):
        # State variables initialization
        self.world = None
        self.client = None
        self.vehicle = None
        self.interval = interval
        self.vehicle_type = vehicle_type
        self.initial_velocity = initial_velocity
        self.spawn_point = spawn_point
        self.logger = logger

    def initialize_carla(self):
        # initialize Carla
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(10.0) # seconds
        
    def initialize_world(self):
        self.world = self.client.load_world("Town05")
        settings = self.world.get_settings()
        settings.fixed_delta_seconds =  self.interval / 1000.0
        settings.substepping = True
        settings.max_substep_delta_time = settings.fixed_delta_seconds / 10
        settings.max_substeps = 10
        settings.synchronous_mode = True # Enables synchronous mode
        self.world.apply_settings(settings)
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

    def get_vehicle_velocity(self):
        return self.vehicle.get_velocity()

    def get_vehicle_position(self):
        return self.gps_queue.get()

    def apply_control(self, throttle, brake):
        self.vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=brake))

    def get_world(self):
        self.world = self.client.get_world()

    def tick(self):
        self.world.tick()

    def initialize_vehicle(self):
        blueprint_library = self.world.get_blueprint_library()        
        
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
        
        # Set the initial velocity
        target_speed = self.initial_velocity
        for i in range(1):
            self.vehicle.set_target_velocity(carla.Vector3D(x=target_speed.x, y=target_speed.y, z=target_speed.z))
            # self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
            # self.world.tick()
        
        self.logger.info("Spawned vehicle")