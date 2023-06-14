from turtlebot import TurtleBot
from environment import BotEnvironment
from supervisor import Supervisor
from ea import EA
import os
import pickle
import numpy as np


def create_pickle_file(input_data, dir_name, file_name):
    """
    Create a pickle file using provided data in the specified directory
    """

    if not os.path.exists(dir_name):  # If Data directory does not exist, create it
        os.makedirs(dir_name)

    path_name = os.path.join(dir_name, file_name)
    rover_file = open(path_name, 'wb')
    pickle.dump(input_data, rover_file)
    rover_file.close()


if __name__ == "__main__":
    # Environment Parameters
    height = 10  # meters
    width = 10  # meters
    obs_radius = 0.5  # Observation radius of target (meters)

    # EA/Test Parameters
    population_size = 25
    generations = 50
    time_steps = 50
    n_configurations = 6  # Number of starting configurations for turtlebot to train on

    # Training configurations
    target_configs = [
        [7.7, 7.7, 0.3048],
        [2.1, 4.2, 0.3048],
        [4.7, 5.3, 0.3048],
        [7.7, 7.7, 0.3048],
        [2.1, 4.2, 0.3048],
        [4.7, 5.3, 0.3048]
    ]
    robot_configs = [
        [5.5, 5.75, 0],
        [4.0, 3.1, 92],
        [2.43, 6.1, 167],
        [5.5, 5.75, 135],
        [4.0, 3.1, 289],
        [2.43, 6.1, 24]
    ]
    hazard_configs = [
        [6.75, 6.7, 0.75],
        [3.0, 3.4, 0.75],
        [3.42, 5.8, 0.75],
        [6.75, 6.7, 0.75],
        [3.0, 3.4, 0.75],
        [3.42, 5.8, 0.75]
    ]

    # Class instances
    sp = Supervisor()
    tb = TurtleBot()
    s_ea = EA(population_size)
    s_ea.create_new_population()
    env = BotEnvironment(height, width, tb)

    for gen in range(generations):
        for pol_id, s_pol in enumerate(s_ea.population):
            bot_reward = 0
            for config_id in range(n_configurations):
                # Reset the robot and get supervisor NN weights
                env.reset_environment(robot_configs[config_id], target_configs[config_id], hazard_configs[config_id])
                sp.get_weights(s_ea.population[s_pol])
                env.hazard_entered = False
                env.target_located = False
                episode_reward = 0
                for t in range(time_steps):
                    # Turtlebot queries LiDAR
                    env.turtlebot.lidar_scan(env)
                    # Supervisor scans environment and generates counterfactual
                    counterfactual = sp.run_supervisor_nn(env.turtlebot.robot_state, env.target, env.hazard)
                    env.turtlebot.get_counterfactual(counterfactual)

                    # Turtlebot takes step
                    env.turtlebot.robot_turn()
                    env.turtlebot.robot_drive()

                    # Check for hazard entry
                    if not env.hazard_entered:
                        env.check_hazard_incursion()

                    # Calculate reward
                    episode_reward = env.calc_agent_reward()
                    if env.target_located:
                        break

                bot_reward += episode_reward

            # Assess fitness of supervisor's policy
            s_ea.fitness[pol_id] = bot_reward/n_configurations

        s_ea.down_select()

    # Save best supervisor policy from EA as pickle file
    best_pol_id = np.argmax(s_ea.fitness)
    best_supervisor_policy = np = s_ea.population[f'pol{best_pol_id}']
    create_pickle_file(best_supervisor_policy, "Supervisor_Networks", "SupervisorNN")

