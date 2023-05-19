class BotEnvironment:
    def __init__(self, height, width, tb):
        self.height = height  # Height is in meters
        self.width = width  # Width is in meters
        self.turtlebot = tb  # Instance of turtlebot class
        self.target = [0, 0, 0]  # Current position and size of the target (size is a radius) [x, y, r]

    def set_target_position(self):
        """
        Set the position for the target the turtlebot must navigate towards
        """

        self.target = [5, 5, 0.5]  # x, y, r  (meters)

    def set_robot_position(self):
        """
        Set the starting position of the robot in the world
        """

        self.turtlebot.loc = [0.5, 0.75]
