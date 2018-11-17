import numpy as np
import sys 
import itertools
import ast

WORMHOLE_FREQUENCY = 3
LASER_DIRECT_ORDER = ['N', 'E', 'S', 'W']
ROBOT = 1
START = 2
END = 3
BARRIER = 10
LASER = 20
BEAM = 30
## Set worm pair with same number 50-50, 51-51 to infinity 
WORM = 50

MAX_CYCLE = 4 ## Define the maximum environment cyclic period

MAX_X = 15
MAX_Y = 15

WORMHOLE_FREQUENCY = 3
LASER_DIRECT_ORDER = ['N', 'E', 'S', 'W']

## searching mode 
GREEDY = 0
EXPLORE = 1


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
        self.start_trial()
        self.distance_tolerance_step = 2
    
    def reset(self):
        self.time = 0
        self.crt_position = self.st_position
        return self.data[:,:,self.time]
    
    def start_trial(self):
        self.trials += 1
        self.time = 0
        self.crt_position = self.st_position
    #    self.routes.append([self.st_position])
        ROUTE_DIC = {
            'ROUTES' : [],
            'STEPS' : self.time,
            'REWARDS' : [0],
            'AVAILABLE_DIRECTION' : [],
            'SELECTED_DIRECTION' : [],
            'DONE' : False
        }
        
        self.routes.append(ROUTE_DIC)
        
        self.greed_search = True
        _, self.nearest_worm_distance = self.get_nearest_wormpairs()
        _,_, self.crt_distance = self.get_dist(self.crt_position, self.ed_position)
        self.shortest_distance = min(self.nearest_worm_distance, self.crt_distance)
        
    def return_env(self, t):
    ## make environment from problem 
    ## set object and lasers and wormhole locations in 2-D np array
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
                    while y+idx < MAX_Y :

                        if data[y+idx, x] != 0:
                            break
                        else:
                            data[y+idx, x] = BEAM
                        idx += 1
                elif direction == 1: # east 
                    while x+idx < MAX_X :
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
    
    def next_available_direction(self, crt_t=0):
    ## return  avaialbe direction at next time step
    ## input - current time 
    ## return ( north, east, south, west ) - each elements is true or false next time available move
    
        crt_x, crt_y = self.crt_position
        next_t_idx = (crt_t+1)%4 

        north = False
        east = False
        south = False
        west = False

        if crt_x+1 < self.m_x :
            if self.data[crt_y, crt_x+1, next_t_idx] < BARRIER or self.data[crt_y, crt_x+1, next_t_idx] >= WORM:
                east = True
        if crt_x-1 >= 0 :
            if self.data[crt_y, crt_x-1, next_t_idx] < BARRIER or self.data[crt_y, crt_x-1, next_t_idx] >= WORM:
                west = True
        if crt_y+1 < self.m_y : 
            if self.data[crt_y+1, crt_x, next_t_idx] < BARRIER or self.data[crt_y+1, crt_x, next_t_idx] >= WORM:
                north = True
        if crt_y-1 >= 0 : 
            if self.data[crt_y-1, crt_x, next_t_idx] < BARRIER or self.data[crt_y-1, crt_x, next_t_idx] >= WORM:
                south = True

        return [north, east, south, west]
    
    def get_nearest_wormpairs(self):
    
    ## find the nearest wormpairs 
    ## input - robot current position, ed_position, list of wormholes pairs
    ## return - nearest wormhole index, nearest wormhole pairs
    
        prev_short_dist = self.m_x*self.m_y*4
        prev_short_dist_idx = -1

        for worm in self.wormholes:
            w1, w2 = worm

            _ , _ , tmp_st_a = self.get_dist(self.crt_position, w1)
            _ , _ , tmp_ed_a = self.get_dist(self.ed_position, w2)

            _ , _ , tmp_st_b = self.get_dist(self.crt_position, w2)
            _ , _ , tmp_ed_b = self.get_dist(self.ed_position, w1)

            if tmp_st_a + tmp_ed_a >= tmp_st_b + tmp_ed_b : 
                crt_short_dist = tmp_st_b + tmp_ed_b
            else: 
                crt_short_dist = tmp_st_a + tmp_ed_a

            if crt_short_dist < prev_short_dist:
                prev_short_dist = crt_short_dist
                prev_short_dist_idx += 1

        worm_idx = prev_short_dist_idx
        nearest_worm_distance = prev_short_dist

        return worm_idx, nearest_worm_distance
    
    def get_dist(self, st, ed):
    ## Get distance two points 
    ## return st-ed each coordinate and absolute distance 
    
        st_x, st_y = st
        ed_x, ed_y = ed

        return ed_x-st_x, ed_y-st_y, abs(ed_x-st_x)+abs(ed_y-st_y)
    
    
    def check_distance_delta(self, direction):
    ## Check the delta of distnace change via direction 
    ## Input : [ north, south, east, west ] in form of [ True, False, False, True ]
    
    
    def get_path(self):
    ## 2018-11-17 Ver => Find Exclusive Path 
    
        crt_x, crt_y = self.crt_position
        
        available_direction = self.next_available_direction(crt_t=self.time)
        
        for item in self.routes:
            prev_
    
    def get_shortes_path_direction(self):
    ## Get direction to goal or nearest wormholes 
        
        rtn_val = -1
        
        worm_idx, nearest_worm_distance = self.get_nearest_wormpairs()
        crt_distance_obj = self.get_dist(self.crt_position, self.ed_position)
        crt_x_dist, crt_y_dist, crt_distance = crt_distance_obj

#        print (nearest_worm_distance, crt_distance)
#        print (crt_x_dist, crt_y_dist)
        
        available_direction = self.next_available_direction(crt_t=self.time)
#        print (self.crt_position, self.ed_position, available_direction)
        
        if worm_idx < 0 or nearest_worm_distance > crt_distance :
            ## no worm or worm distance is longer than ed_position distance
            short_direction = [crt_y_dist>=0, crt_x_dist>=0, crt_y_dist<0, crt_x_dist<0]
            
        else:
            crt_distance_obj = self.get_dist(self.crt_position, self.wormholes[worm_idx])
            crt_x_dist, crt_y_dist, crt_distance = crt_distance_obj
            short_direction = [crt_y_dist>=0, crt_x_dist>=0, crt_y_dist<0, crt_x_dist<0]
        
        
        short_next_direction = list(np.multiply(available_direction, short_direction))
        
        if np.sum(short_next_direction) == 0:
            print ('Random Choice step :',self.time)
            #idx_rtv = []
            
            crt_x, crt_y = self.crt_position
            next_dist_array = [self.get_dist( (crt_x, crt_y+1), self.ed_position), 
                               self.get_dist( (crt_x+1, crt_y), self.ed_position),
                               self.get_dist( (crt_x, crt_y-1), self.ed_position),
                               self.get_dist( (crt_x-1, crt_y), self.ed_position) ]
            
            min_idx = np.argmin(np.multiply(next_dist_array, available_direction))
            
            
            #idx_rtv = [i for i, e in enumerate(available_direction) if e == True]
            #print (available_direction)
            #np.random.shuffle(idx_rtv)
            print (min_idx)
            rtn_val = min_idx #idx_rtv[0]
            
        else:
            print (short_next_direction)
            rtn_val = short_next_direction.index(True)

        return rtn_val
    
    def chk_search_mode(self):
        ## Same Coord 
        
        return
    
    def next_step(self):
        crt_x, crt_y = self.crt_position
        next_x = crt_x
        next_y = crt_y
        path_direction = self.get_shortes_path_direction()
        if path_direction == 0:
            next_y += 1
        elif path_direction == 1:
            next_x += 1 
        elif path_direction == 2:
            next_y -= 1 
        elif path_direction == 3:
            next_x -= 1 
        else : 
            return False

        return next_x, next_y
    
    def add_route(self):
        
        
        next_x, next_y = self.next_step()
        self.routes[self.trials].append((next_x, next_y)) 
        self.crt_position = (next_x, next_y)
        self.time += 1 
        
        return self.check_end()
    
    def check_end(self):
        
        if self.ed_position == self.crt_position:
            print ('Robot arrived')
            return True
        else:
            return False
        
    def step(self, action):
        
        done = False
        info = 0
        
        x, y = self.crt_position
        next_t_idx = (self.time+1)%4 
        data = self.data[:,:,next_t_idx]
        if action == 0:
            self.crt_position = (x, y+1)
        elif action == 1:
            self.crt_position = (x+1, y)
        elif action == 2:
            self.crt_position = (x, y-1)
        elif action == 3:
            self.crt_position = (x-1, y)
        
        ## check agent in wormholes 
        for worms in self.wormholes:
            w1, w2 = worms
            if self.crt_position == w1 :
                self.crt_position = w2
            elif self.crt_position == w2 :
                self.crt_position = w1
        
        ## Check agent is in the end position 
        self.time += 1
        if self.ed_position == self.crt_position:
            done = True 
            reward = 100
        else:
            reward = -1
        
        ## return new observation, 
        return data, reward, done, info
    
    def render(self):
        crt_t_idx = (self.time)%4 
        data = np.copy(self.data[:,:,crt_t_idx])
        x, y = self.crt_position
        data[y, x] = ROBOT
        print (data)
            
        
        
robot_env = Env(file_name = './problem3/problem.txt')
robot_env.routes