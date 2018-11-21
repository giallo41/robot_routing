#!/usr/bin/env python
import numpy as np
import random
import readchar
import env
import argparse

LEFT = 3 # West -> x decrease
DOWN = 0 # North -> y increase
RIGHT = 1 # East -> x increase
UP = 2 # South -> y decrease

arrow_keys = {
    '\x1b[A' : UP,
    '\x1b[B' : DOWN,
    '\x1b[C' : RIGHT,
    '\x1b[D' : LEFT
}

def main():
    parser = argparse.ArgumentParser(description='Play with robot enviromnet')
    parser.add_argument('file', type=str, default='problem.txt',
            help='Problem file')
    args = parser.parse_args()

    file_name = args.file

    routes = []
    rewards = 0

    robot_env = env.Env(file_name = file_name)
    robot_env.render()
    print ('Start Loc :', robot_env.st_position)
    print ('End loc :', robot_env.ed_position)
    print ('Input Arrow key to move')
    routes.append(robot_env.st_position)
    while True:
        key = readchar.readkey()

        if key not in arrow_keys.keys():
            print("Wrong Key - Game aborted!")
            break

        action = arrow_keys[key]
        state, reward, done, info = robot_env.step(action)
        routes.append( ( int(state%robot_env.m_x), int(state/robot_env.m_x) ) )
        print ('Steps : ',robot_env.time)
        print("Routes :", routes)
        robot_env.render()
        print (' ')
        rewards += reward

        if done:
            print("Finished with reward: ", rewards)
            print("Steps :", robot_env.time)
            print("Routes :", routes)
            break



if __name__=="__main__":
    main()
