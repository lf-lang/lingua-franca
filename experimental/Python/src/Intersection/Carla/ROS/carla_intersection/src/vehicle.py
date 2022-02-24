from src.utils import make_speed, dotdict, distance
from src.constants import BILLION, GOAL_REACHED_THRESHOLD, GOAL_REACHED_THRESHOLD_TIME, SPEED_LIMIT

class Vehicle:
    def __init__(self, vehicle_id, initial_position, initial_velocity, clock, logger):
        self.vehicle_id = vehicle_id
        self.position = initial_position
        self.velocity = initial_velocity
        self.speed = make_speed(initial_velocity)
        self.goal_reached = False
        self.granted_time_to_enter = 0
        self.intersection_position = None
        self.clock = clock
        self.logger = logger

    def set_velocity(self, new_velocity):
        self.velocity = new_velocity
        self.speed = make_speed(new_velocity)
        # Prevent divisions by zero
        if self.speed == 0.0:
            self.speed = 0.001

    def set_position(self, new_position):
        self.position = new_position

    def get_velocity(self):
        return self.velocity
    
    def get_position(self):
        return self.position

    def get_speed(self):
        return self.speed

    def grant(self, arrival_time, intersection_position):
        self.logger.info("Vehicle {} Granted access".format(self.vehicle_id + 1) + 
            "to enter the intersection at elapsed logical time {:d}.\n".format(
                int(arrival_time)
            )
        )
        self.granted_time_to_enter = arrival_time
        self.intersection_position = intersection_position

    def receive_velocity_from_simulator(self, vehicle_stat):
        pub_packets = dotdict()
        if self.goal_reached:
            return pub_packets
        # Record the speed
        self.set_velocity(vehicle_stat)
        
        # Send a new request to the RSU if no time to enter
        # the intersection is granted
        if self.granted_time_to_enter == 0:
            pub_packets.request = dotdict()
            pub_packets.request.requestor_id = self.vehicle_id
            pub_packets.request.speed = self.get_speed()
            pub_packets.request.position = self.get_position()

            # Stop the vehicle
            pub_packets.cmd = dotdict()
            pub_packets.cmd.throttle = 0.0
            pub_packets.cmd.brake = 1.0
        else:
            # We have a granted time from the RSU
            # All we need to do is adjust our velocity
            # to enter the intersection at the allocated
            # time
            
            # First, how far are we from the intersection
            distance_remaining = distance(self.intersection_position, self.get_position())
            current_time = self.clock.get_current_time_in_ns()
            time_remaining = (self.granted_time_to_enter - current_time) / BILLION
            
            self.logger.info("########################################")
            self.logger.info("Vehicle {}: Distance to intersection: {}m.".format(self.vehicle_id, distance_remaining))
            self.logger.info("Vehicle {}: Time to intersection: {}s.".format(self.vehicle_id, time_remaining))
            self.logger.info("Vehicle {}: Current speed: {}m/s.".format(self.vehicle_id, self.speed))

            target_speed = 0.0
            # target_speed = distance_remaining/time_remaining
            
            if distance_remaining <= GOAL_REACHED_THRESHOLD and \
                    time_remaining <= GOAL_REACHED_THRESHOLD_TIME :
                # Goal reached
                # At this point, a normal controller should stop the vehicle until
                # it receives a new goal. However, for the purposes of this demo,
                # it will set the target speed to the speed limit so that vehicles
                # can leave the intersection (otherwise, they will just stop at the
                # intersection).
                target_speed = SPEED_LIMIT
                # Simulation is over
                self.goal_reached = True
                
                self.logger.info("\n\n*************************************************************\n\n".format(self.vehicle_id))
                self.logger.info("************* Vehicle {}: Reached intersection! *************".format(self.vehicle_id))
                self.logger.info("\n\n*************************************************************\n\n".format(self.vehicle_id))

            elif time_remaining < (distance_remaining / SPEED_LIMIT):
                # No time to make it to the intersection even if we
                # were going at the speed limit.
                # Ask the RSU again
                self.granted_time_to_enter = 0
                # Apply the brake since we ran out of time
                target_speed = 0
            else:
                # Has not reached the goal
                # target_speed = ((2 * distance_remaining) / (time_remaining)) - self.speed
                target_speed = distance_remaining / time_remaining
            
            self.logger.info("Vehicle {}: Calculated target speed: {}m/s.".format(self.vehicle_id, target_speed))
            
            if (target_speed - SPEED_LIMIT) > 0:
                self.logger.info("Warning: target speed exceeds the speed limit")
                target_speed = 0
                self.granted_time_to_enter = 0
            
            if target_speed <= 0:
                self.logger.info("Warning: target speed negative or zero")
                target_speed = 0.001
                self.granted_time_to_enter = 0
            
            brake = 0.0
            throttle = 0.0
            
            if target_speed >= self.speed:            
                # Calculate a proportional throttle (0.0 < throttle < 1.0)
                throttle = min((target_speed - self.speed)/target_speed, 1.0)
                # throttle = 1.0
                brake = 0.0
                # throttle = min(abs(target_speed / self.speed), 1)
            else:
                # Need to apply the brake
                brake = min((self.speed - target_speed)/self.speed, 1.0)
                # brake = 1.0
                throttle = 0.0
            
            # Check throttle boundaries
            if throttle < 0:
                self.logger.info("Error: negative throttle")
                throttle = 0.0
            
            # Prepare and send the target velocity as a vehicle command
            pub_packets.cmd = dotdict()
            pub_packets.cmd.throttle = throttle
            pub_packets.cmd.brake = brake
            self.logger.info("Vehicle {}: Throttle: {}. Brake: {}".format(self.vehicle_id, throttle, brake))
        return pub_packets


