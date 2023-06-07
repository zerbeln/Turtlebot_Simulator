import math


class BotEnvironment:
    def __init__(self, height, width, tb):
        self.height = height  # Height is in meters
        self.width = width  # Width is in meters
        self.turtlebot = tb  # Instance of turtlebot class
        self.target = [0, 0, 0]  # Current position and size of the target (size is a radius) [x, y, r]
        self.hazard = [0, 0, 0]  # Current positona nd size of the hazard (size is a radius) [x, y, r]

    def set_target_position(self):
        """
        Set the position for the target the turtlebot must navigate towards
        """

        self.target = [7.7, 7.7, 0.3048]  # x, y, r  (meters)

    def set_robot_position(self):
        """
        Set the starting position of the robot in the world
        """

        self.turtlebot.robot_state = [5.5, 5.75, 0]  # x, y, theta (meters, degrees)

    def set_hazard_position(self):
        """
        Set the location of the hazard in the environment
        """

        self.hazard = [6.75, 6.7, 0.75]  # x, y, r (meters)

    def check_hazard_incursion(self):
        """
        Checks to see if turtlebot has entered the hazardous area
        """
        x_dist = abs(self.turtlebot.robot_state[0] - self.hazard[0])
        y_dist = abs(self.turtlebot.robot_state[1] - self.hazard[1])
        total_dist = math.sqrt(x_dist**2 + y_dist**2)

        if total_dist < self.hazard[2]:  # See if turtlebot within hazard radius
            return True
        else:
            return False

    def reset_environment(self):
        """
        Reset the environment to initial conditions
        """

        self.set_robot_position()
        self.set_target_position()

