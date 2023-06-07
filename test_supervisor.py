from turtlebot import TurtleBot
from environment import BotEnvironment
from supervisor import Supervisor
from main import run_visualizer
import math
import os
import pickle


def load_saved_supervisor_policies(file_name):
    """
    Load saved Neural Network from pickle file
    """

    dir_name = f'Supervisor_Networks/'
    fpath_name = os.path.join(dir_name, file_name)
    weight_file = open(fpath_name, 'rb')
    weights = pickle.load(weight_file)
    weight_file.close()

    return weights


if __name__ == "__main__":
    # Environment Parameters
    height = 10  # meters
    width = 10  # meters
    time_steps = 50
    obs_radius = 0.5  # Observation radius of target (meters)

    # Data
    tb_trajectory = []  # Trajectory of bot

    # Class instances
    sp = Supervisor()
    tb = TurtleBot()
    env = BotEnvironment(height, width, tb)
    env.set_hazard_position()
    env.set_target_position()
    env.set_robot_position()
    tb_trajectory.append(env.turtlebot.robot_state)

    # Load Supervisor Network Weights
    nn_weights = load_saved_supervisor_policies("SupervisorNN")
    sp.get_weights(nn_weights)

    hazard_entered = False
    target_located = False
    for t in range(time_steps):
        # Turtlebot queries LiDAR
        env.turtlebot.lidar_scan(env)
        # Supervisor scans environment and generates counterfactual
        counterfactual = sp.run_supervisor_nn(env.turtlebot.robot_state, env.target, env.hazard)
        env.turtlebot.get_counterfactual(counterfactual)

        # Turtlebot takes step
        env.turtlebot.robot_turn()
        env.turtlebot.robot_drive()
        tb_trajectory.append(env.turtlebot.robot_state)

        # Check hazard incursion
        haz_check = env.check_hazard_incursion()
        if haz_check:
            hazard_entered = True

        x_dist = env.turtlebot.robot_state[0] - env.target[0]
        y_dist = env.turtlebot.robot_state[1] - env.target[1]
        total_dist = math.sqrt(x_dist ** 2 + y_dist ** 2)

        if total_dist < obs_radius:
            target_located = True
            break

    if hazard_entered:
        print("Robot travelled through hazard")

    run_visualizer(width, height, tb_trajectory, env.target, hazard=env.hazard)

