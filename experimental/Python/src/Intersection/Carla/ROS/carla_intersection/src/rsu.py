# Other libraries
from utils import distance, dotdict

# Constants
from constants import BILLION

class RSU:
    def __init__(self, intersection_width, nominal_speed_in_intersection, intersection_position, clock, logger):
        self.intersection_width = intersection_width
        self.nominal_speed_in_intersection = nominal_speed_in_intersection
        self.intersection_position = intersection_position
        self.earliest_free = 0 # nsec
        self.active_participants = [0] * 20
        self.clock = clock
        self.logger = logger


    def receive_request(self, request) -> dotdict:
        '''
        Handles the request to enter the intersection from a vehicle. 

        Parameters
        ----------
        request: Class
            A request to enter the intersection from a 
            vehicle with the following structure:
            - requestor_id : int
            - speed : float
            - position : coordinate

        Returns
        ----------
        dotdict
            A dotted dictionary with the following structure:
            - grant
                - requestor_id : int
                - intersection_position : coordinate
                - target_speed : float
                - arrival_time : int
        '''
        pub_packets = dotdict()
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
        time_to_intersection = dr / speed_in_m_per_sec 

        self.logger.info("*** RSU: Vehicle {}'s distance to intersection is {}. ".format(request.requestor_id, dr, self.intersection_position, request.position, time_to_intersection))
    
        current_time = self.clock.get_current_time_in_ns()
        
        # Convert the time interval to nsec (it is in seconds).
        arrival_time_ns = int(current_time + (time_to_intersection * BILLION))
        
        pub_packets.grant = dotdict()
        pub_packets.grant.requestor_id = request.requestor_id
        pub_packets.grant.intersection_position = self.intersection_position
        if arrival_time_ns >= self.earliest_free:
            # Vehicle can maintain speed.
            pub_packets.grant.target_speed = request.speed
            pub_packets.grant.arrival_time = arrival_time_ns
        else:
            # Could be smarter than this, but just send the nominal speed in intersection.
            pub_packets.grant.target_speed = self.nominal_speed_in_intersection
            # Vehicle has to slow down and maybe stop.
            pub_packets.grant.arrival_time = self.earliest_free
        
        # Update earliest free on the assumption that the vehicle
        # maintains its target speed (on average) within the intersection.
        time_in_intersection_ns = int((BILLION * self.intersection_width) / (pub_packets.grant.target_speed))
        self.earliest_free = pub_packets.grant.arrival_time + time_in_intersection_ns
        
        self.logger.info("*** RSU: Granted access to vehicle {} to enter at "
            "time {} ns with average target velocity {} m/s. Next available time is {}".format(
            pub_packets.grant.requestor_id,
            pub_packets.grant.arrival_time,
            pub_packets.grant.target_speed,
            self.earliest_free)
        )
        return pub_packets