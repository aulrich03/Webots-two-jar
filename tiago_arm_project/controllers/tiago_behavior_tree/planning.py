import py_trees
import numpy as np
from heapq import heapify, heappush, heappop
from collections import defaultdict


# Function to convert world coordinates to map coordinates

def world2map(xw, yw):
    px = (200/4 * (xw + 2.3))
    py = (300/6 * -(yw - 1.7))
    px = min(px, 199)
    py = min(py, 299)
    px = max(px, 0)
    py = max(py, 0)
    return [int(px), int(py)]
    
# Function to convert map coordinates to world coordinates

def map2world(px, py):
    xw = px * (4/200) - 2.3
    yw = py * (6/300) - 1.7
    return [xw, -yw]
    
    
# A* grid search algorithm

def getShortestPath(map, start, goal):
    
    # Returns all of the valid neighbors of the input node along with their respective edge costs
    
    def getNeighbors(u):
        neighbors = []
        for delta in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, -1), (-1, 1), (1, 1), (-1, -1)):
            cand = (u[0] + delta[0], u[1] + delta[1])
            if (cand[0] >= 0 and cand[0] < len(map) and cand[1] >= 0 and cand[1] < len(map[0]) and map[cand[0]][cand[1]] < 0.3):
                neighbors.append((np.sqrt(delta[0]**2+delta[1]**2), cand))
        return neighbors
    
    
    queue = [(0, start)]
    heapify(queue)
    
    distances = defaultdict(lambda:float("inf"))
    distances[start] = 0
    
    visited = {start}
    parent = {}
    goalFound = False
    
    # Uses priority queue and euclidian heuristic to only visit relevant nodes
    
    while queue and goalFound == False:
        (currentdist, v) = heappop(queue)
        visited.add(v)
        for (costvu, u) in getNeighbors(v):
            if u not in visited:
                heuristic = np.sqrt((goal[0]-u[0])**2+(goal[1]-u[1])**2)
                newcost = distances[v] + costvu
                if newcost < distances[u]:
                    distances[u] = newcost
                    heappush(queue, (newcost + heuristic, u))
                    parent[u] = v
            if u == goal:
                goalFound = True
                break

    key = goal
    path = []
    while key in parent.keys():
        key = parent[key]
        path.insert(0,key)
    path.append(goal)
    
    return path
    
    
    
# Create Planning subclass

class Planning(py_trees.behaviour.Behaviour):
    
    # Initialize a full Node instance plus the blackboard element and goal element
    
    def __init__(self, name, blackboard, goal):
        super(Planning, self).__init__(name)
        self.robot = blackboard.retrieve('robot')
        self.blackboard = blackboard
        px, py = world2map(goal[0], goal[1])
        
        self.goal = (px, py)
        
    # Set up necessary values for planning
    
    def setup(self):
        self.timestep = 32
        
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        self.display = self.robot.getDevice('display')
        
    # Calculate the shortest path to the goal and draw the cspace created in the mapping step
    # on the robot display
    
    def initialise(self):
            
    
        
        
        
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        
        sx, sy = world2map(xw, yw)
        self.start = (sx, sy)
        self.map = np.load('cspace.npy')
        
        
        self.best_path = getShortestPath(self.map, self.start, self.goal)
        self.robot_path = np.array([map2world(i[0], i[1]) for i in self.best_path])
        
        self.blackboard.add('waypoints', self.robot_path)
        
        self.display.setColor(0xFFFFFF)
        for i in range(len(self.map)):
            for j in range(len(self.map[i])):
                if self.map[i][j] == True:
                    self.display.drawPixel(i, j)
    
    # After initialization there are no updates to be made
    
    def update(self):
        

        
                    
        return py_trees.common.Status.SUCCESS
        
        
    # Draw the shortest path to the goal on the robot display in blue
    
    def terminate(self, new_status):
        self.display.setColor(0x0000FF)
        for i in self.best_path:
            self.display.drawPixel(i[0], i[1])
        print(self.name)
        
        
        
            
            
            
    