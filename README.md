# Robot Routing

----------------------

## Finding the solution of routing Problem

created by Sue Hun Jung giallo.hos@gmail.com


#### Files

```
robot.py
env.py
play.py
Dockerfile
```

#### Requirements
```
pip install -r requirements.txt
```

#### Run
```
./robot.py problem.txt solution.txt
```

#### docker build and run
```
# Build docker image
docker build -t robot-img:latest .

# Run docker container
docker run -it --entrypoint=/bin/bash robot-img:latest

# Run solver file
./robot.py problem.txt solution.txt
./robot.py problem1/problem.txt problem1/solution.txt
./robot.py problem2/problem.txt problem2/solution.txt
./robot.py problem3/problem.txt problem3/solution.txt
./robot.py problem4/problem.txt problem4/solution.txt

```


#### Details

**1)    `robot.py`**

- Find the shortest path within the number of 'episodes'
- optional arguments
  - --max_x : max grid sizes of x (default : 15)
  - --max_y : max grid sizes of y (default : 15)
  - --episodes : number of episodes to iterate (default : 10,000)
  - --e_steps : number of Step to explore and decay (default : 1,000)
  - --l_rate : learning rate (default : 0.85)
  - --discount : discount rate of rewards (default : 0.99)<br>
- make the solution file with list of tuples eg
`[(1,1), (1,2) (1,3)]`
- If there is no solution to find make the empty list file eg
`[]`

**2)    `env.py`**

- Creating Environment for routing problem
- `env.reset()` : reset the environment for next trials ( episodes ), return - current state
- `env.step(action)` : input the available action then return the tuple (next_state, reward, done, info)

**3)    `play.py`**

- Playing with the problem
- Display the problem
- Input key with keyboards arrows (up, down, left, right)
- Run : `./play.py problem.txt`

```
example play

[[ 0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.]
 [ 0.  0.  0.  0.  0.  0.  0. 10.  3.  0. 50.  0.  0.  0.  0.]
 [ 0.  0. 50.  0.  0.  0.  0.  0. 10.  0.  0.  0.  0.  0.  0.]
 [ 0.  0.  1.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.]
 [ 0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.]
 [ 0.  0.  0.  0. 10.  0.  0.  0.  0. 20.  0.  0.  0.  0.  0.]
 [ 0.  0.  0.  0.  0.  0.  0.  0.  0. 30.  0.  0.  0.  0.  0.]
 [ 0.  0.  0.  0.  0.  0.  0.  0.  0. 30.  0.  0.  0.  0.  0.]
 [ 0.  0.  0.  0.  0.  0.  0.  0.  0. 30.  0.  0.  0.  0.  0.]
 [ 0.  0.  0.  0.  0.  0.  0.  0.  0. 30.  0.  0.  0.  0.  0.]
 [ 0.  0.  0.  0.  0.  0.  0.  0.  0. 30.  0.  0.  0.  0.  0.]
 [ 0.  0.  0.  0.  0.  0.  0.  0.  0. 30.  0.  0.  0.  0.  0.]
 [ 0.  0.  0.  0.  0.  0.  0.  0.  0. 30.  0.  0.  0.  0.  0.]
 [ 0.  0.  0.  0.  0.  0.  0.  0.  0. 30.  0.  0.  0.  0.  0.]
 [ 0.  0.  0.  0.  0.  0.  0.  0.  0. 30.  0.  0.  0.  0.  0.]]
Start Loc : (2, 3)
End loc : (8, 1)
Input Arrow key to move

0 - empty node (cell)
1 - Robot
2 - Start Position
3 - End Position
10 - Block ( Obstacle )
20 - Laser location ( Obstacle )
30 - Laser (Beam) ( Obstacle )
50+ - Wormholes pair ( 50 - 50 ), ( 51 - 51 ), (52 - 52) ...

```


--------------------


### FAQ
 - If `./robot.py` & `./play.py` occurs Permission Denied Error
  `chmod 755 robot.py`, `chmod 755 play.py`
