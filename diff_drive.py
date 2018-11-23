import time
import math
from pprint import pprint, pformat

def clamp(minimum, x, maximum):
    # Clamp the value of x to be between minimum and maximum
    return max(minimum, min(x, maximum))

def shortest_angular_difference(firstAngle, secondAngle):
    difference = secondAngle - firstAngle
    while (difference < -180):
        difference += 360
    while (difference > 180):
        difference -= 360
    return difference

class RobotDiffDriveController:
    '''
    A simple P-controller for differential drive robots
    '''

    x_current_pose = None
    y_current_pose = None
    current_heading = None  # Degrees

    def __init__(self, **kwargs):

        # P-controller settings
        self.kP_lin = kwargs.get('kP_lin', 0.05)
        self.kP_ang = kwargs.get('kP_ang', 0.01)

        # Velocity Constraints
        self.max_lin_vel = kwargs.get('max_lin_vel', 1.0)
        self.max_ang_vel = kwargs.get('max_ang_vel', 0.0)
        
        self.max_driver_power = kwargs.get('max_driver_power', 1.0)
        self.min_driver_power = kwargs.get('min_driver_power', 0.2)

        # Stop moving if we are in a radius of reached_target_threshold metres from the target
        self.reached_target_threshold = kwargs.get('reached_target_threshold', 1.0)

        self.verbose = kwargs.get('verbose', False)

        print('Starting the differential drive controller with the following settings: ')
        pprint(kwargs)
        time.sleep(3.0)

    def position_update(self, x, y):
        self.x_current_pose = x
        self.y_current_pose = y

        if self.verbose:
            print('Robot is at position: ({:.2f}, {:.2f})'.format(x, y))

    def heading_update(self, heading_degrees):
        self.current_heading = heading_degrees

        if self.verbose:
            print('Robot has heading: ({:.2f})'.format(heading_degrees))

    def get_wheel_velocities(self, x_desired, y_desired):
        for varname in ['x_current_pose', 'y_current_pose', 'current_heading']:
            if not(varname in vars(self)):
                raise Exception('{} has not been set. Robot cannot drive without this'.format(varname))

        delta_x = self.x_current_pose - x_desired
        delta_y = self.y_current_pose - y_desired

        # Pythagoras to find distance to target
        dist_to_target = math.sqrt(math.pow(delta_x,2) + math.pow(delta_y,2))

        # Trigonometry
        heading_to_target_deg = math.atan2(delta_y, delta_x) * (180.0 / math.pi)
        
        # Compute Shortest Angular Difference in degrees
        delta_heading = shortest_angular_difference(self.current_heading, heading_to_target_deg)

        if self.verbose:
            print('Robot target position set to: ({:.2f}, {:.2f}). Robot has to travel {:.2f} metres, turning {:.2f} degrees'.format(x_desired, y_desired, dist_to_target, delta_heading))
        
        # Convert heading difference back to radians
        desired_heading_rad = delta_heading * (math.pi / 180.0)

        linear_velocity = self.kP_lin * dist_to_target
        angular_velocity = self.kP_ang * delta_heading

        if self.verbose:
            print('LIN_VEL: {:.2f}, ANG_VEL: {:.2f}'.format(linear_velocity, angular_velocity))

        # Limit The Velocities
        linear_velocity = clamp(0, linear_velocity, self.max_lin_vel)
        angular_velocity = clamp(-self.max_ang_vel, angular_velocity, self.max_ang_vel)

        # Calculate the motor speeds
        if dist_to_target > self.reached_target_threshold:
            left_speed = linear_velocity - angular_velocity
            right_speed = linear_velocity + angular_velocity
            print('Unconstrained speeds: ({:.2f}, {:.2f})'.format(left_speed, right_speed))
        else:
            left_speed, right_speed = 0.0, 0.0
            print('REACHED TARGET!')

        # Limit the motor speeds to what the driver can handle
        left_speed = clamp(-self.max_driver_power, left_speed, self.max_driver_power)
        right_speed = clamp(-self.max_driver_power, right_speed, self.max_driver_power)

        # Cut power if below threshold
        # This avoids motor windings burning out
        if abs(left_speed) < self.min_driver_power:
            left_speed = 0.0

        if abs(right_speed) < self.min_driver_power:
            right_speed = 0.0

        return (left_speed, right_speed)
