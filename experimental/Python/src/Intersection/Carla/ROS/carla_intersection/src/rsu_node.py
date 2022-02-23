# ROS2 libraries
import rclpy
from rclpy.node import Node

# ROS2 messages
from carla_intersection_msgs.msg import Request, Grant

# Other libraries
from src.utils import distance, make_coordinate, make_Vector3

# Constants
from src.constants import BILLION


class RSU(Node):
    def __init__(self):
        super().__init__("rsu")

        # Parameters declaration
        self.declare_parameter('intersection_width', 0)
        self.declare_parameter('nominal_speed_in_intersection', 0.0)
        self.declare_parameter('intersection_position', [0.0, 0.0, 0.0])

        # State variables initialization
        self.intersection_width = self.get_parameter('intersection_width').value
        self.nominal_speed_in_intersection = self.get_parameter('nominal_speed_in_intersection').value
        self.intersection_position = make_coordinate(self.get_parameter('intersection_position').value)
        self.earliest_free = 0 # nsec
        self.active_participants = [0] * 20

        # pubsub for input / output ports
        self.grant_ = self.create_publisher(Grant, "grant", 10)
        self.request_ = self.create_subscription(Request, "request", self.request_callback, 10)

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
        dr = distance(self.intersection_position, request.position)
        arrival_time_sec = dr / speed_in_m_per_sec 

        self.get_logger().info("*** RSU: Vehicle {}'s distance to intersection is {}. ".format(request.requestor_id+1, dr, self.intersection_position, request.position, arrival_time_sec))
    
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
            response.target_speed = self.nominal_speed_in_intersection
            # Vehicle has to slow down and maybe stop.
            response.arrival_time = self.earliest_free
        
        response.intersection_position = make_Vector3(self.intersection_position)
        response.requestor_id = request.requestor_id
        self.grant_.publish(response)
        # Update earliest free on the assumption that the vehicle
        # maintains its target speed (on average) within the intersection.
        time_in_intersection_ns = int((BILLION * self.intersection_width) / (response.target_speed))
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