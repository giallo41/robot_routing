import numpy as np
import random
import readchar
import env

routes = []
rewards = 0

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

robot_env = env.Env(file_name = './problem2/problem.txt')
#print (robot_env.routes)
#print (robot_env.routes[0][0].coord)
robot_env.render()
print ('Start Loc :', robot_env.st_position)
print ('End loc :', robot_env.ed_position)
routes.append(robot_env.st_position)
while True:
    key = readchar.readkey()

    if key not in arrow_keys.keys():
        print("Wrong Key - Game aborted!")
        break
    
    action = arrow_keys[key] 
    state, reward, done, info = robot_env.step(action) 
    routes.append( ( int(state%MAX_X), int(state/MAX_X) ) )
    print ('Steps : ',robot_env.time) 
    robot_env.render()
    print (' ')
    rewards += reward
    
    if done: 
        print("Finished with reward: ", rewards)
        print("Steps :", robot_env.time)
        print("Routes :", routes)
        break