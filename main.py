from turtlebot import TurtleBot
from environment import BotEnvironment
import math
import turtle
import time


def run_visualizer(width, height, tb_trajectory, target):
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
    tg.color("red")
    tg.shape("circle")
    tg.shapesize(15/20)
    tx = ((target[0]/width) * screen_width) - (screen_width/2)
    ty = ((target[1]/height) * screen_height) - (screen_height/2)
    tg.goto(tx, ty)
    tg.stamp()

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
    time_steps = 500
    tb_trajectory = []  # Trajectory of bot

    tb = TurtleBot()
    env = BotEnvironment(height, width, tb)

    env.set_target_position()
    env.set_robot_position()
    env.turtlebot.lidar_scan(env)
    tb_trajectory.append(env.turtlebot.loc)

    x_dist = env.turtlebot.loc[0] - env.target[0]
    y_dist = env.turtlebot.loc[1] - env.target[1]
    total_dist = math.sqrt(x_dist**2 + y_dist**2)
    # print("Distance From Target: ", total_dist)

    for t in range(time_steps):
        env.turtlebot.robot_turn()
        env.turtlebot.robot_drive()
        tb_trajectory.append(env.turtlebot.loc)
        env.turtlebot.lidar_scan(env)

        x_dist = env.turtlebot.loc[0] - env.target[0]
        y_dist = env.turtlebot.loc[1] - env.target[1]
        total_dist = math.sqrt(x_dist ** 2 + y_dist ** 2)
        # print("Distance From Target: ", total_dist)

        if total_dist < env.target[2]:
            break

    run_visualizer(width, height, tb_trajectory, env.target)
