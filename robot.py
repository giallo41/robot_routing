import numpy as np
import env
import random
import argparse

def main():
    
    parser = argparse.ArgumentParser(description='Play with robot enviromnet')
    parser.add_argument('--file', type=str, default='problem.txt',
            help='Problem file')
    args = parser.parse_args()
    
    parser.add_argument('--max_x', type=int, default=15,
        help='Max X size of grids')
    
    parser.add_argument('--max_y', type=int, default=15,
        help='Max Y size of grids')
    
    parser.add_argument('--episodes', type=int, default=10000,
        help='Number of episodes to play')
    
    parser.add_argument('--e_steps', type=int, default=1000,
        help='Number of Step to explore and decay')
    
    parser.add_argument('--l_rate', type=float, default=0.85,
        help='Learning Rate')
    
    parser.add_argument('--discount', type=float, default=0.99,
        help='Learning Rate')
    
    args = parser.parse_args()
        
    file_name = args.file
    max_x = args.max_x
    max_y = args.max_y
    num_episodes = args.episodes
    decay_step = args.e_steps
    l_rate = args.l_rate
    discount = args.discount
    
 
    print ("> Hyper parameter ")
    print ("> File-name :",file_name)
    print ("> Max X grid :", max_x)
    print ("> Max Y grid :", max_y)
    print ("> Num episode :", num_episodes)
    print ("> Decay step :", decay_step)
    print ("> Learning Rate :", l_rate)
    print ("> Discount Rate :", discount)
    
    
    # Load Robot walking Environment
    robot_env = env.Env(file_name = file_name, m_x=max_x, m_y=max_y)
    print ("----- Problem Grid ------")
    robot_env.render()
    print ("> St point :", robot_env.st_position, "Ed point :", robot_env.ed_position)

    # Q-space ( Size of Grid , Action ) 
    Q = np.zeros([(max_x+1)*(max_y+1), 4])
    
    # Store Rewards 
    rewards_list = []
    states_list = []
    
    # Set Max steps to add ( to avoid infinite steps ) 
    max_steps = max_x*max_y
    
    print ('>>> Now finding possible routes from learning <<<')
    for i in range(num_episodes):
        # Reset environment and get first new observation
        state = robot_env.reset()
        rewards = 0
        done = False

        # Random Selection Prob decay during episodes    
        e = 1. / ((i // decay_step) + 1)  

        # The Q-Table learning algorithm
        steps = 0
        while not done and steps < max_steps:
            
            if np.random.rand(1) < e:
                # Randomly Select the direction ( Exploration )
                action = random.choice([0,1,2,3])
            else:
                # Exploitation 
                action = np.argmax(Q[state, :])

            # Get new state and reward from environment
            new_state, reward, done, _ = robot_env.step(action)

            # Update Q-Table with new knowledge using learning rate
            Q[state,action]= Q[state,action]+l_rate*(reward+discount* np.max(Q[new_state,:])-Q[state,action])

            rewards += reward
            state = new_state
            states_list.append(state)

        rewards_list.append(rewards)
    
    selected_states = list(set(states_list))
    tmp1 = Q[selected_states,:]==0 
    tmp2 = Q[selected_states,:]<=-100 
        
    # From Total Routes, Find the Completed Routes
    route_len = 0
    done_route = []
    for item in robot_env.routes:
        if item[-1] == robot_env.ed_position:
            done_route.append(item)
    
    # Check Exist solution 
    if len(done_route)==0 and np.sum(np.multiply(tmp1, tmp2)) == 0 :
        print ("No Solution") 
    else :
        # To find the shortes path solutions from completed routes
        idx = 0
        route_steps = max_x * max_y
        min_step_idx = 0
        for item in done_route:
            if len(item) < route_steps :
                route_steps = len(item)
                min_step_idx = idx
            idx += 1
        print ('> Min steps & index :', route_steps, min_step_idx)
        print ('> Solution : ')
        print (done_route[min_step_idx])

    
if __name__=="__main__":
    main()