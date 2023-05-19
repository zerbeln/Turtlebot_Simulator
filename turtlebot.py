import numpy as np
import math


class TurtleBot:
    def __init__(self):
        self.loc = [0, 0]  # Current position of robot
        self._iloc = [0, 0]  # Initial position of robot (used for reset)
        self.lidar_data = np.zeros(360)  # scans 360 degree radius
        self.resolution = 1  # degrees
        self.sensitivity = 100  # how many intervals each lidar beam is broken into (for "ray tracing")
        self.scan_dist = 10  # m
        self.max_vel = 0.3  # m/s
        self.max_turn = 5  # deg/s
        self.speed = 0.1  # m/s
        self.heading = 0  # robot heading with respect to global frame (degrees)

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
        object_direction = np.argmin(self.lidar_data)  # Closes ping in lidar scan

        if 0 < object_direction <= 180:
            self.heading += self.max_turn
        elif 180 < object_direction < 360:
            self.heading -= self.max_turn

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
