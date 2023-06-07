from turtlebot import TurtleBot
from environment import BotEnvironment
import math
import turtle
import time
import numpy as np


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
        haz.shapesize(25 / 20)
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

    env.set_target_position()
    env.set_robot_position()
    env.set_hazard_position()
    env.turtlebot.lidar_scan(env)
    tb_trajectory.append(env.turtlebot.robot_state)

    x_dist = env.turtlebot.robot_state[0] - env.target[0]
    y_dist = env.turtlebot.robot_state[1] - env.target[1]
    total_dist = math.sqrt(x_dist**2 + y_dist**2)

    for t in range(time_steps):
        env.turtlebot.robot_turn()
        env.turtlebot.robot_drive()
        tb_trajectory.append(env.turtlebot.robot_state)
        env.turtlebot.lidar_scan(env)

        x_dist = env.turtlebot.robot_state[0] - env.target[0]
        y_dist = env.turtlebot.robot_state[1] - env.target[1]
        total_dist = math.sqrt(x_dist ** 2 + y_dist ** 2)

        if total_dist < env.target[2]:
            break

    run_visualizer(width, height, tb_trajectory, env.target, hazard=env.hazard)
