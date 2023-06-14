import math


class BotEnvironment:
    def __init__(self, height, width, tb):
        self.height = height  # Height is in meters
        self.width = width  # Width is in meters
        self.turtlebot = tb  # Instance of turtlebot class
        self.target = [0, 0, 0]  # Current position and size of the target (size is a radius) [x, y, r]
        self.hazard = [0, 0, 0]  # Current positona nd size of the hazard (size is a radius) [x, y, r]
        self.hazard_entered = False  # Flag that indicates whether the turtlebot entered a hazard or not
        self.target_located = False  # Flag that indicates whether the turtlebot found the target or not

    def set_target_position(self, t_config):
        """
        Set the position for the target the turtlebot must navigate towards
        """

        self.target = t_config  # x, y, r  (meters)

    def set_robot_position(self, r_config):
        """
        Set the starting position of the robot in the world
        """

        self.turtlebot.robot_state = r_config  # x, y, theta (meters, degrees)

    def set_hazard_position(self, h_config):
        """
        Set the location of the hazard in the environment
        """

        self.hazard = h_config  # x, y, r (meters)

    def check_hazard_incursion(self):
        """
        Checks to see if turtlebot has entered the hazardous area
        """
        x_dist = abs(self.turtlebot.robot_state[0] - self.hazard[0])
        y_dist = abs(self.turtlebot.robot_state[1] - self.hazard[1])
        total_dist = math.sqrt(x_dist**2 + y_dist**2)

        if total_dist < self.hazard[2]:  # See if turtlebot within hazard radius
            self.hazard_entered = True

    def calc_agent_reward(self):
        """
        Calculate the turtlebot's reward
        """
        reward = 0

        # Check if the turtlebot entered a hazard
        if self.hazard_entered:
            reward -= 10

        # Check if turtlebot within range of target
        x_dist = self.turtlebot.robot_state[0] - self.target[0]
        y_dist = self.turtlebot.robot_state[1] - self.target[1]
        dist = math.sqrt(x_dist**2 + y_dist**2)

        if dist < self.target[2] + 0.1:
            if dist < 1:
                dist = 1
            self.target_located = True
            reward += 5/dist

        return reward

    def reset_environment(self, r_config, t_config, h_config):
        """
        Reset the environment to initial conditions
        """

        self.set_robot_position(r_config)
        self.set_target_position(t_config)
        self.set_hazard_position(h_config)

