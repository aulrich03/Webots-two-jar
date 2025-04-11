import py_trees
import numpy as np
from scipy import signal
from matplotlib import pyplot as plt


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
    
    
    

# Create Mapping subclass

class Mapping(py_trees.behaviour.Behaviour):
    
    # Initialize a full Node instance plus the blackboard element
    
    def __init__(self, name, blackboard):
        super(Mapping, self).__init__(name)
        
        
        self.hasrun = False
        self.robot = blackboard.retrieve('robot')
        
    #Set up sensors and display
    
    def setup(self):
        self.timestep = 32
        
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
        self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        
        self.display = self.robot.getDevice('display')
        
        self.kernel = np.ones((30,30))
        
        print("Begin Mapping")
        
    # Initialize the np arrays that are used for converting lidar readings into points in robot
    # coordinates
    
    def initialise(self):
        self.map = np.zeros((200,300))
        self.angles = np.linspace(120*np.pi/180, -120*np.pi/180, 667)
        self.angles = self.angles[80:-80]
        
        
    # Mapping step
    
    def update(self):
        self.hasrun = True
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])
        
        
        # Draw the robot trajectory on the robot display in red
        
        px, py = world2map(xw, yw)
        self.display.setColor(0xFF0000)
        self.display.drawPixel(px, py)
        
        # Use numpy matrix multiplication to transform robot coordinates into world coordinates
        
        w_T_r = np.array([[np.cos(theta), -np.sin(theta), xw],
                         [np.sin(theta), np.cos(theta), yw],
                         [0, 0, 1]])
                         
                         
        ranges = np.array(self.lidar.getRangeImage())
        ranges = ranges[80:-80]
        ranges[ranges == np.inf] = 100
        X_i = np.array([ranges*np.cos(self.angles)+0.202, ranges*np.sin(self.angles), np.ones((507,))])
        D = w_T_r @ X_i
        
        
        
        # Increment probabilistic map values with increasingly bright color
        
        for d in D.transpose():
            px, py = world2map(d[0], d[1])
            self.map[px,py]+=0.01
            if self.map[px,py] > 1:
                self.map[px, py] = 1
            v=int(self.map[px,py]*255)
            color=(v*256**2+v*256+v)
            self.display.setColor(int(color))
            self.display.drawPixel(px,py)
            
            
        return py_trees.common.Status.RUNNING
        
        
    def terminate(self, new_status):
        
        # After mapping is complete, display a map of the robot configuration space by 
        # convolving every point on the map with a probabilistic value greater than 0.9
        # with a 30x30 kernel
        
        if(self.hasrun):
            cmap = signal.convolve2d(self.map,self.kernel,mode='same')
            cspace = cmap>0.9
            np.save('cspace',cspace)
            plt.imshow(cspace)
            plt.show()
        
        print("Mapping complete")