import numpy as np
import math


class TurtleBot:
    def __init__(self):
        self.loc = [0, 0]  # Current position of robot
        self._iloc = [0, 0]  # Initial position of robot (used for reset)
        self.lidar_data = np.zeros(360)  # scans 360 degree radius
        self.resolution = 1  # degrees
        self.sensitivity = 100  # how many intervals each lidar beam is broken into (for "ray tracing")
        self.scan_dist =  3.5 # m
        self.max_vel = 0.22  # m/s
        self.max_turn = 2.84 # rad/s
        self.speed = 0.1  # m/s
        self.heading = 0  # robot heading with respect to global frame (degrees)
        self.turn_p = 0.003 # Proprtional constant for Proportional control for turning

    def lidar_scan(self, env):
        """
        Robot scans environment to collect data
        """
        # Robot position
        xo = self.loc[0]
        yo = self.loc[1]

        # Target position
        xt = env.target[0]
        yt = env.target[1]
        rt = env.target[2]

        # Perform scan
        scan_angle = 0
        while scan_angle < 360:
            object_found = False
            for i in range(self.sensitivity):
                # Determine current scan depth
                s_dist = i * (self.scan_dist/self.sensitivity)
                xi = s_dist*math.cos((self.heading+scan_angle)*math.pi/180)
                yi = s_dist*math.sin((self.heading+scan_angle)*math.pi/180)

                # Look for "contact" with target
                scan_loc = [xo+xi, yo+yi]  # Scan "waypoint"
                dist_target = math.sqrt((xt - scan_loc[0])**2 + (yt - scan_loc[1])**2)
                if dist_target < rt:
                    self.lidar_data[scan_angle] = s_dist
                    object_found = True
                    break

            if not object_found:
                self.lidar_data[scan_angle] = self.scan_dist
            scan_angle += self.resolution

    def robot_turn(self):
        """
        Robot turns towards object
        """
        # The index of the poi in the lidar scan is the nearest range
        poi_ind = np.argmin(self.lidar_data)  # Closes ping in lidar scan

        # Move poi_ind from [0,360] frame to [-180,180] frame
        poi_ind_adjusted = poi_ind - 180

        # Calculate angle difference (degrees)
        if poi_ind_adjusted >= 0:
            angle_diff = poi_ind_adjusted-180
        else:
            angle_diff = 180+poi_ind_adjusted

        # Threshold (degrees) for when the robot can stop the turn and just drive straight
        diff_threshold = 2

        # If we're within the threshold, then stop turning
        if abs(angle_diff) <= diff_threshold:
            turn_rad = 0.
        # Else outside the threshold. Calculate turn
        else:
            # Calculate turn
            turn_rad = angle_diff*self.turn_p

            # Bound based on max velocity
            if turn_rad > self.max_turn:
                turn_rad = self.max_turn
            elif turn_rad < -self.max_turn:
                turn_rad = - self.max_turn

        # Heading is in degrees.
        # I need to convert radians to degrees before adding to heading
        turn_deg = turn_rad*57.2958

        self.heading += turn_deg

    def robot_drive(self):
        """
        Robot drives forward at current velocity and heading
        """
        xo = self.loc[0]
        yo = self.loc[1]

        delta_x = self.speed * math.cos(self.heading*math.pi/180)
        delta_y = self.speed * math.sin(self.heading*math.pi/180)

        xt = xo + delta_x
        yt = yo + delta_y

        self.loc = [xt, yt]
