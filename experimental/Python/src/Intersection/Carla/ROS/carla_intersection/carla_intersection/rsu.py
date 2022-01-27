import rclpy
from rclpy.node import Node

from math import sin, cos, sqrt, atan2, radians, pi

try:
    from math import isclose
except ImportError:
    def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
        return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

from carla_intersection_msgs.msg import Request, Grant
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

class RSU(Node):
    def __init__(self):
        super().__init__("rsu")

        self.intersection_width = self.declare_parameter('intersection_width', 0)
        self.nominal_speed_in_intersection = self.declare_parameter('nominal_speed_in_intersection', 0.0)
        self.intersection_position = self.declare_parameter('intersection_position', [0.0, 0.0, 0.0])
        
        self.earliest_free = 0 # nsec
        self.active_participants = [0] * 20

        # pubsub for input / output ports
        self.grant_ = self.create_publisher(Grant, "grant", 10)
        self.request_ = self.create_subscription(Request, "request", self.request_callback, 10)

    def get_intersection_position(self):
        p = self.intersection_position.value
        return Vector3(x=p[0], y=p[1], z=p[2])

    def get_nominal_speed_in_intersection(self):
        return float(self.nominal_speed_in_intersection.value)

    def get_intersection_width(self):
        return int(self.intersection_width.value)


    def request_callback(self, request):              
        self.active_participants[request.requestor_id] = 1
        if request.speed == 0.0:
            # Avoid division by zero
            request.speed = 0.001
        # Calculate the time it will take the approaching vehicle to
        # arrive at its current speed. Note that this is
        # time from the time the vehicle sends the message
        # according to the arriving vehicle's clock.
        speed_in_m_per_sec = request.speed
        dr = distance(self.get_intersection_position(), request.position)
        self.get_logger().info("*** RSU: Vehicle {}'s distance to intersection is {}m.".format(request.requestor_id+1, dr))
        arrival_time_sec = dr / speed_in_m_per_sec 
    
        time_message_sent = self.get_clock().now().to_msg()
        
        # Convert the time interval to nsec (it is in seconds).
        arrival_time_ns = int(time_message_sent.sec * BILLION + time_message_sent.nanosec + (arrival_time_sec * BILLION))
        
        response = Grant()
        if arrival_time_ns >= self.earliest_free:
            # Vehicle can maintain speed.
            response.target_speed = request.speed
            response.arrival_time = arrival_time_ns
        else:
            # Could be smarter than this, but just send the nominal speed in intersection.
            response.target_speed = self.get_nominal_speed_in_intersection()
            # Vehicle has to slow down and maybe stop.
            response.arrival_time = self.earliest_free
        
        response.intersection_position = self.get_intersection_position()
        response.requestor_id = request.requestor_id
        self.grant_.publish(response)
        # Update earliest free on the assumption that the vehicle
        # maintains its target speed (on average) within the intersection.
        time_in_intersection_ns = int((BILLION * self.get_intersection_width()) / (response.target_speed))
        self.earliest_free = response.arrival_time + time_in_intersection_ns
        
        self.get_logger().info("*** RSU: Granted access to vehicle {} to enter at "
            "time {} ns with average target velocity {} m/s. Next available time is {}".format(
            response.requestor_id + 1,
            response.arrival_time,
            response.target_speed,
            self.earliest_free)
        )

def main(args=None):
    rclpy.init(args=args)

    rsu = RSU()

    rclpy.spin(rsu)

    # Destroy the node explicitly
    rsu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()