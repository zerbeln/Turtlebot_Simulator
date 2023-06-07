from turtlebot import TurtleBot
from environment import BotEnvironment
from supervisor import Supervisor
from ea import EA
import math
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
    time_steps = 50
    obs_radius = 0.5  # Observation radius of target (meters)

    # EA Parameters
    population_size = 20
    generations = 10

    # Class instances
    sp = Supervisor()
    tb = TurtleBot()
    s_ea = EA(population_size)
    s_ea.create_new_population()
    env = BotEnvironment(height, width, tb)
    env.set_hazard_position()
    env.set_target_position()

    for gen in range(generations):
        for pol_id, s_pol in enumerate(s_ea.population):
            # Reset the robot and get supervisor NN weights
            env.set_robot_position()
            sp.get_weights(s_ea.population[s_pol])

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

            # Assess fitness of supervisor's policy
            if hazard_entered:
                s_ea.fitness[pol_id] = -10
            elif target_located and not hazard_entered:
                s_ea.fitness[pol_id] = 1
            else:
                s_ea.fitness[pol_id] = 0

        s_ea.down_select()

    # Save best supervisor policy from EA as pickle file
    best_pol_id = np.argmax(s_ea.fitness)
    best_supervisor_policy = np = s_ea.population[f'pol{best_pol_id}']
    create_pickle_file(best_supervisor_policy, "Supervisor_Networks", "SupervisorNN")

