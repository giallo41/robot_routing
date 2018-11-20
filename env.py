import numpy as np
import sys 
import ast

WORMHOLE_FREQUENCY = 3
LASER_DIRECT_ORDER = ['N', 'E', 'S', 'W']
ROBOT = 1
START = 2
END = 3
BARRIER = 10
LASER = 20
BEAM = 30
WORM = 50 ## Set worm pair with same number 50-50, 51-51 to infinity 

MAX_CYCLE = 4 ## Define the maximum environment cyclic period

MAX_X = 15
MAX_Y = 15

def open_problem(file_name = 'problem.txt'):
    file_contents = []
    try: 
        with open(file_name) as f:
            file_contents = f.readlines()
        file_contents = [line.strip() for line in file_contents]
    except:
        print ('Error - Open File does not exsit :', file_name)
        return

    barriers, static_lasers, rotating_lasers, wormhole_pairs = [], [], [], []
    
    st_point = ast.literal_eval(file_contents[0])
    ed_point = ast.literal_eval(file_contents[1])
    barriers = ast.literal_eval(file_contents[2])
    lasers_tmp = ast.literal_eval(file_contents[3])
    wormholes = ast.literal_eval(file_contents[4])
    
    lasers = []
    if len(lasers_tmp) > 0:
        for item in lasers_tmp:
            x, y, direction = item
            lasers.append( (x, y, LASER_DIRECT_ORDER.index(direction) ) )
            
    
    ## Wormholes have pairs [ [ (), ()], [(),() ]]
    return st_point, ed_point, barriers, lasers, wormholes
    
    
class Env:
    def __init__(self, file_name = './problem1/problem.txt', m_x=MAX_X, m_y=MAX_Y, m_cycle=MAX_CYCLE):
        
        self.m_x=m_x
        self.m_y=m_y
        self.m_cycle=m_cycle
        
        self.st_position, self.ed_position, self.barriers, self.lasers, self.wormholes = open_problem(file_name = file_name)
        
        self.data = np.zeros((m_y, m_x, m_cycle))
    
        for i in range(m_cycle):
            self.data[:,:,i] = self.return_env(i)
        
        self.routes = []
        self.trials = -1
        self.reset()

    def reset(self):
        self.time = 0
        self.trials += 1
        self.crt_position = self.st_position
        self.routes.append([self.crt_position])
        x, y = self.crt_position
        return x+self.m_x*y
    
    def return_env(self, t):
        """
        make environment from problem 
        set object and lasers and wormhole locations in 2-D np array

        input : current time 
        return : 2-d grid with object at time t 
        """

        data = np.zeros((self.m_y, self.m_x))

        try:

            for item in self.barriers:
                x, y = item 
                data[y,x] = BARRIER

            if t%WORMHOLE_FREQUENCY == 0 :
                pair_idx = 0
                for pair_wormhole in self.wormholes:
                    for item in pair_wormhole:
                        x, y = item 
                        data[y,x] = WORM + pair_idx
                    pair_idx += 1

            for item in self.lasers:
                x, y, direction = item
                data[y,x] = LASER

                direction = (direction + t)%4
                idx = 1
                if direction == 0: # north 
                    while y+idx < self.m_y :

                        if data[y+idx, x] != 0:
                            break
                        else:
                            data[y+idx, x] = BEAM
                        idx += 1
                elif direction == 1: # east 
                    while x+idx < self.m_x :
                        if data[y, x+idx] != 0:
                            break
                        else:
                            data[y, x+idx] = BEAM
                        idx += 1
                elif direction == 2: # south
                    while y-idx >= 0  :
                        if data[y-idx, x] != 0:
                            break
                        else:
                            data[y-idx, x] = BEAM
                        idx += 1
                elif direction == 3: # west
                    while x-idx >= 0 :
                        if data[y, x-idx] != 0:
                            break
                        else:
                            data[y, x-idx] = BEAM
                        idx += 1
            x, y = self.st_position
            data[y,x] = START
            x, y = self.ed_position
            data[y,x] = END
        except:
            print ('Data Max Size Error, Set the new maximum size. Current MAX_X={}, MAX_Y={}'.format(self.max_x,self.max_y))

        return np.array(data, dtype = int)

    
    def next_available(self):
        """
        4way possible direction at current node 
        input : None
        return : possible way index ( 0 - north, 1 - east, 2 - south, 3- west), [ True, False, True, False ] 
        """
        crt_t = self.time
        crt_x, crt_y = self.crt_position
        next_t_idx = (crt_t+1)%4 

        north = False
        east = False
        south = False
        west = False
        
        rtn = []

        if crt_x+1 < self.m_x :
            if self.data[crt_y, crt_x+1, next_t_idx] < BARRIER or self.data[crt_y, crt_x+1, next_t_idx] >= WORM:
                east = True
                rtn.append(1)
        if crt_x-1 >= 0 :
            if self.data[crt_y, crt_x-1, next_t_idx] < BARRIER or self.data[crt_y, crt_x-1, next_t_idx] >= WORM:
                west = True
                rtn.append(3)
        if crt_y+1 < self.m_y : 
            if self.data[crt_y+1, crt_x, next_t_idx] < BARRIER or self.data[crt_y+1, crt_x, next_t_idx] >= WORM:
                north = True
                rtn.append(0)
        if crt_y-1 >= 0 : 
            if self.data[crt_y-1, crt_x, next_t_idx] < BARRIER or self.data[crt_y-1, crt_x, next_t_idx] >= WORM:
                south = True
                rtn.append(2)

        return rtn, [north, east, south, west]

    def step(self, action):
        
        done = False
        info = 0
        reward = -1 # each step will rewared -1 ( to minimize the path ) 
        
        x, y = self.crt_position
        next_t_idx = (self.time+1)%4 
        data = self.data[:,:,next_t_idx]
        if action == 0:
            tmp_next_position = (x, y+1)
        elif action == 1:
            tmp_next_position = (x+1, y)
        elif action == 2:
            tmp_next_position = (x, y-1)
        elif action == 3:
            tmp_next_position = (x-1, y)
        
        ## check agent next move reward 
        n_x, n_y = tmp_next_position
        
        ## Step 1 : out-of play ground 
        if n_x < 0 or n_x >= self.m_x or n_y < 0 or n_y >= self.m_y :
            reward = -1000
            done = True
        else:
            next_node = data[n_y, n_x]
        ## Object - Index 
        #ROBOT = 1
        #START = 2
        #END = 3
        #BARRIER = 10
        #LASER = 20
        #BEAM = 30
        #WORM = 50+ 

            if next_node == END : # Robot arrived at End Position 
                done = True 
                reward = 1000
            elif BARRIER == next_node or next_node == LASER or next_node == BEAM : # Object 
                done = True 
                reward = -1000
            elif 50 <= next_node : # Wormholes 
                for worms in self.wormholes:
                    w1, w2 = worms
                    if tmp_next_position == w1 :
                        tmp_next_position = w2
                    elif tmp_next_position == w2 :
                        tmp_next_position = w1
        
        ## Next State Update 
        self.crt_position = tmp_next_position        
        self.time += 1
        self.routes[self.trials].append(self.crt_position) # add routes 

        ## return new observation, reward, done, info ( info is not using ) 
        x, y = self.crt_position
        return x+self.m_x*y, reward, done, info
        #return data, reward, done, info
        
    def render(self):    
    ## Display Play ground at current time
        crt_t_idx = (self.time)%4 
        data = np.copy(self.data[:,:,crt_t_idx])
        x, y = self.crt_position
        data[y, x] = ROBOT
        print (data)
