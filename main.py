from turtlebot import TurtleBot
from environment import BotEnvironment
import math
import turtle
import time
import numpy as np
import math


def run_visualizer(width, height, tb_trajectory, target, hazard=None):
    # Define Screen parameters
    screen_height = width * 50
    screen_width = height * 50
    screen = turtle.Screen()
    screen.setup(screen_width+20, screen_height+20)
    screen.title("Turtlebot Sim")
    screen.bgcolor("blue")
    screen.tracer(0)  # Turns off automatic animations

    # Define Target
    tg = turtle.Turtle()
    tg.color("green")
    tg.shape("circle")
    tg.shapesize(15/20)
    tg.penup()
    tx = ((target[0]/width) * screen_width) - (screen_width/2)
    ty = ((target[1]/height) * screen_height) - (screen_height/2)
    tg.goto(tx, ty)
    tg.stamp()

    # Define hazard
    if hazard is not None:
        haz = turtle.Turtle()
        haz.color("red")
        haz.shape("circle")
        haz_area = 0.5*math.pi*(0.75**2)
        haz.shapesize(haz_area*90 / 20)
        haz.penup()
        hx = ((hazard[0] / width) * screen_width) - (screen_width / 2)
        hy = ((hazard[1] / height) * screen_height) - (screen_height / 2)
        haz.goto(hx, hy)
        haz.stamp()


    # Define Turtlebot
    tb = turtle.Turtle()
    tb.color("white")
    tb.shape("triangle")
    tb.shapesize(5/20)
    tb.penup()

    for loc in tb_trajectory:
        bx = ((loc[0]/width) * screen_width) - (screen_width / 2)
        by = ((loc[1]/height) * screen_height) - (screen_height / 2)
        tb.goto(bx, by)
        tb.stamp()
        screen.update()
        time.sleep(0.2)

    turtle.done()


if __name__ == "__main__":
    height = 10  # meters
    width = 10  # meters
    time_steps = 50
    tb_trajectory = []  # Trajectory of bot

    tb = TurtleBot()
    env = BotEnvironment(height, width, tb)

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

    env.set_target_position(target_configs[5])
    env.set_robot_position(robot_configs[5])
    env.set_hazard_position(hazard_configs[5])
    env.turtlebot.lidar_scan(env)
    tb_trajectory.append(env.turtlebot.robot_state)

    x_dist = env.turtlebot.robot_state[0] - env.target[0]
    y_dist = env.turtlebot.robot_state[1] - env.target[1]
    total_dist = math.sqrt(x_dist**2 + y_dist**2)

    env.hazard_entered = False
    env.target_located = False
    for t in range(time_steps):
        env.turtlebot.robot_turn()
        env.turtlebot.robot_drive()

        if not env.hazard_entered:
            env.check_hazard_incursion()
        else:
            print("HAZARD")

        tb_trajectory.append(env.turtlebot.robot_state)
        env.turtlebot.lidar_scan(env)

        x_dist = env.turtlebot.robot_state[0] - env.target[0]
        y_dist = env.turtlebot.robot_state[1] - env.target[1]
        total_dist = math.sqrt(x_dist ** 2 + y_dist ** 2)

        if total_dist < env.target[2] + 0.1:
            env.hazard_entered = False
            env.check_hazard_incursion()
            break

    if env.hazard_entered:
        print("HAZARD POI")
    run_visualizer(width, height, tb_trajectory, env.target, hazard=env.hazard)
