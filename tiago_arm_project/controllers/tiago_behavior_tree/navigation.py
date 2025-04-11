import py_trees
import numpy as np

# Create Navigation behavior

class Navigation(py_trees.behaviour.Behaviour):
    
    # Initialize a full Node instance plus the blackboard element
    
    def __init__(self, name, blackboard):
        super(Navigation, self).__init__(name)
        self.robot = blackboard.retrieve('robot')
        self.blackboard = blackboard
        
    # Set up the necessary features of the robot stored in the blackboard    
        
    def setup(self):
        self.timestep = 32
        
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
        self.leftMotor = self.robot.getDevice('wheel_left_joint')
        self.rightMotor = self.robot.getDevice('wheel_right_joint')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        
        
        self.MAX_SPEED = self.blackboard.retrieve('MAX_SPEED')
        self.marker = self.robot.getFromDef("marker").getField("translation")
        
        
    # Initialization step    
    
    def initialise(self):
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
        
        self.index = 0
        
        self.WP = self.blackboard.retrieve("waypoints")
        
        
    # Use a proportional controller to navigate to a set of waypoints read in from
    # the blackboard
    
    def update(self):
    
        
        
       
        self.marker.setSFVec3f([*self.WP[self.index], 0])
        
        # Get values used to compute values for proportional controller
        
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        
        theta=np.arctan2(self.compass.getValues()[0],self.compass.getValues()[1])
        
        rho = np.sqrt((self.WP[self.index][0] - xw)**2 + (self.WP[self.index][1] - yw)**2)
        alpha = np.arctan2(self.WP[self.index][1] - yw, self.WP[self.index][0] - xw) - theta
        
        if alpha > np.pi:
            alpha = alpha - 2 * np.pi
        elif alpha < -np.pi:
            alpha = alpha + 2 * np.pi
            
          
        # Set speeds using a proportional controller that is 4 times the angle between
        # the robot and the waypoint and 2 times the distance between the robot and the waypoint
        
        v_left, v_right = self.MAX_SPEED, self.MAX_SPEED
        
        p1 = 4
        
        p2 = 2
        
        v_left = -p1 * alpha + p2 * rho
        v_right = p1 * alpha + p2 * rho
        
        v_left = min(v_left, self.MAX_SPEED)
        v_right = min(v_right, self.MAX_SPEED)
        v_left = max(v_left, -self.MAX_SPEED)
        v_right = max(v_right, -self.MAX_SPEED)
        
        self.leftMotor.setVelocity(v_left)
        self.rightMotor.setVelocity(v_right)
        
        
        
        
        # Iterate to next waypoint until the last waypoint is reached
        
        if rho < 0.4:
            self.index += 1
            if self.index == len(self.WP):
                print("Destination reached")
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.RUNNING
            
            
    # Update circuit_finished on blackboard if the final destination has been reached        
    
    def terminate(self, new_status):
        print("Destination reached")