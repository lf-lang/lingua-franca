# ROS2 libraries
import rclpy
from rclpy.node import Node

# ROS2 messages
from carla_intersection_msgs.msg import Request, Grant

# Other libraries
from src.utils import distance, make_coordinate, make_Vector3, ROSClock
from src.rsu import RSU

class RSUNode(Node):
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
        
        # pubsub for input / output ports
        self.grant_ = self.create_publisher(Grant, "grant", 10)
        self.request_ = self.create_subscription(Request, "request", self.request_callback, 10)
        self.rsu = RSU(intersection_width = self.intersection_width,
                       nominal_speed_in_intersection = self.nominal_speed_in_intersection,
                       intersection_position = self.intersection_position,
                       clock = ROSClock(self.get_clock()),
                       logger = self.get_logger())


    def request_callback(self, request):              
        pub_packets = self.rsu.receive_request(request)
        if pub_packets.grant != None:
            grant = Grant()
            grant.requestor_id = pub_packets.grant.requestor_id
            grant.intersection_position = make_Vector3(pub_packets.grant.intersection_position)
            grant.target_speed = pub_packets.grant.target_speed
            grant.arrival_time = pub_packets.grant.arrival_time
            self.grant_.publish(grant)


def main(args=None):
    rclpy.init(args=args)

    rsu = RSUNode()

    rclpy.spin(rsu)

    # Destroy the node explicitly
    rsu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()