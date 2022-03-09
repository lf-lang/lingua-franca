BILLION = 1_000_000_000

CARLA_INSTALL_DIR = "/opt/carla-simulator"

# The speed limit of vehicles in m/s
SPEED_LIMIT = 14.0

# The distance (in meters) at which the controller assumes it has reached its goal
GOAL_REACHED_THRESHOLD = 14.0

# The time threshold at which the vehicle has reached its time-based goal
GOAL_REACHED_THRESHOLD_TIME = (GOAL_REACHED_THRESHOLD / SPEED_LIMIT)
