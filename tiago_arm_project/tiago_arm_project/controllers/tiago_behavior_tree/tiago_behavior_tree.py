from os.path import exists
import py_trees
from controller import Robot, Supervisor
from matplotlib import pyplot as plt
import numpy as np
from py_trees.composites import Sequence, Parallel, Selector
from scipy import signal

from navigation import Navigation
from mapping import Mapping
from planning import Planning
from camera import CameraScan
from inverse_kinematics import TestIkpy
from position_arm import PositionArm
# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = 32


MAX_SPEED = 6.28

# Waypoints used to circumnavigate the kitchen for mapping

WP = [(-0.189, 0.0543), (0.114, 0.0624), (0.536, -0.425), (0.536, -1.48), (0.374, -2.89),
         (-1.73, -3.03), (-1.71, -1.5), (-1.6, 0.03), (-0.06, 0.4)]

# Joint encoder positions that result in a safe arm position for navigation

safe_position = {
       'torso_lift_joint' : 0.1,
       'arm_1_joint' : 0.71,
       'arm_2_joint' : 1.02,
       'arm_3_joint' : -2.815,
       'arm_4_joint' : 1.011,
       'arm_5_joint' : 0,
       'arm_6_joint' : 0,
       'arm_7_joint' : 0,
       'gripper_left_finger_joint' : 0,
       'gripper_right_finger_joint' : 0,
       'head_1_joint' : 0,
       'head_2_joint' : 0}
       
       
# Joint position that allows for a smooth dropping of the honey jar       

drop_position = {
       'torso_lift_joint' : 0.1,
       'arm_1_joint' : 0.71,
       'arm_2_joint' : 0.45,
       'arm_3_joint' : -1.3,
       'arm_4_joint' : 1.011,
       'arm_5_joint' : 0,
       'arm_6_joint' : 0,
       'arm_7_joint' : 0.5,
       'gripper_left_finger_joint' : 0,
       'gripper_right_finger_joint' : 0,
       'head_1_joint' : 0,
       'head_2_joint' : 0}


# py_trees behvaiour to close finger joints when picking up objects

class CloseGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(CloseGripper, self).__init__(name)
        self.blackboard = blackboard
        self.robot = blackboard.retrieve('robot')
        
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.gripper_left = self.robot.getDevice('gripper_left_finger_joint')
        self.gripper_right = self.robot.getDevice('gripper_right_finger_joint')
        
        self.gripper_left.enableForceFeedback(timestep)
        self.gripper_right.enableForceFeedback(timestep)
        
        closed = False
        self.index = 0
        
    
    def update(self):
        
        print("Closing")
        
        if self.name == "Close blue_jam":
            self.gripper_left.setPosition(0)
            self.gripper_right.setPosition(0)
            
        else:
            self.gripper_left.setPosition(0)
        
        self.left_force = self.gripper_left.getForceFeedback()
        self.right_force = self.gripper_left.getForceFeedback()
        
        closed = self.left_force <= -10
        
        self.index += 1
        print(self.robot.getDevice('gripper_left_sensor_finger_joint').getValue())
        
        
        if closed == False:
            return py_trees.common.Status.RUNNING
        else:
            print("Closed", self.left_force, self.right_force)
            return py_trees.common.Status.SUCCESS
      
            
    def terminate(self, new_status):
        self.blackboard.add('waypoints', [(0.14, -1.83)])
        
# Lowers robot down to pick up honey jar using the torso joint        

class LowerRobot(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(LowerRobot, self).__init__(name)
        self.robot = blackboard.retrieve('robot')
        
        
    def update(self):
        
        self.robot.getDevice('torso_lift_joint').setPosition(0)
        
        
        if self.robot.getDevice('torso_lift_joint_sensor').getValue() <= 0.0001:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
            
# Raises robot up after picking up honey jar using torso joint            

class RaiseRobot(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(RaiseRobot, self).__init__(name)
        self.robot = blackboard.retrieve('robot')
        
        
    def update(self):
        
        self.robot.getDevice('torso_lift_joint').setPosition(0.1)
        
        
        if self.robot.getDevice('torso_lift_joint_sensor').getValue() >= 0.099:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
            
            
            
    
 # Opens finger joints to drop jars onto the table       

class OpenGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(OpenGripper, self).__init__(name)
        self.robot = blackboard.retrieve('robot')
        
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.gripper_left = self.robot.getDevice('gripper_left_finger_joint')
        self.gripper_right = self.robot.getDevice('gripper_right_finger_joint')
        
        self.left_encoder = self.robot.getDevice('gripper_left_sensor_finger_joint')
        self.right_encoder = self.robot.getDevice('gripper_right_sensor_finger_joint')
        self.index = 0
        
        
        open = False
        
    def update(self):
        
        
        left_position = self.left_encoder.getValue()
        right_position = self.right_encoder.getValue()
        
        error = ((0.045 - left_position) + (0.045 - right_position))**2 <= 0.0001 
        
        self.gripper_left.setPosition(0.045)
        self.gripper_right.setPosition(0.045)
        
        closed = error
        
        if self.index >= 50:
            if closed == False:
                return py_trees.common.Status.RUNNING
            else:
                return py_trees.common.Status.SUCCESS
        else:
            self.index += 1
            return py_trees.common.Status.RUNNING
            
            
            
    def terminate(self, new_status):
       done = True


class MicroMovement(py_trees.behaviour.Behaviour):
    
    def __init__(self, name, blackboard, direction, steps):
        super(MicroMovement, self).__init__(name)
        self.direction = direction
        self.blackboard = blackboard
        self.robot = blackboard.retrieve('robot')
        self.steps = steps
        
        
    def setup(self):
        self.leftWheel = self.robot.getDevice('wheel_left_joint')
        self.rightWheel = self.robot.getDevice('wheel_right_joint')
        
        self.leftWheel.setPosition(float('inf'))
        self.rightWheel.setPosition(float('inf'))
        
    def initialise(self):
       
            
        self.index = 0
        
    def update(self):
    
        if self.direction == 'forward':
            self.leftWheel.setVelocity(0.6)
            self.rightWheel.setVelocity(0.6)
        elif self.direction == 'backward':
            self.leftWheel.setVelocity(-1.6)
            self.rightWheel.setVelocity(-1.6)
        elif self.direction == 'right':
            self.leftWheel.setVelocity(1.6)
            self.rightWheel.setVelocity(-1.6)
        
        if self.index >= self.steps:
            return py_trees.common.Status.SUCCESS
        else:
            self.index += 1
            return py_trees.common.Status.RUNNING
            
    def terminate(self, new_status):
    
        done = True
        self.leftWheel.setVelocity(0)
        self.rightWheel.setVelocity(0)


class doesMapExist(py_trees.behaviour.Behaviour):

    def update(self):
    
        file_exists = exists('cspace.npy')
        if file_exists:
            print("Map already exists")
            return py_trees.common.Status.SUCCESS
        else:
            print("Map does not exist")
            return py_trees.common.Status.FAILURE
            
# Simple blackboard implementation used by professor that stores relevant data
# in a dictionary          

class Blackboard:
    def __init__(self):
        self.data = {}
    def add(self, key, value):
        self.data[key] = value
    def retrieve(self, key):
        return self.data.get(key)
       
# Initialize values in the blackboard

blackboard = Blackboard()
blackboard.add('robot',robot)
blackboard.add('waypoints',np.concatenate((WP,np.flip(WP,0)),axis=0))
blackboard.add('circuit_finished', False)
blackboard.add('MAX_SPEED', MAX_SPEED)
blackboard.add('new_position', safe_position)
blackboard.add('honey_position', (1.16, 0.479))
blackboard.add('safe_position', safe_position)
blackboard.add('drop_position', drop_position)
# Tests to see if all waypoints have been reached to allow the program to exit

class isCircuitFinished(py_trees.behaviour.Behaviour):

    def __init__(self, name, blackboard):
        super(isCircuitFinished, self).__init__(name)
        
        self.blackboard = blackboard
        
    
    def update(self):
        circuitFinished = self.blackboard.retrieve('circuit_finished')
        
        if circuitFinished:
            print("Circuit complete")
            return py_trees.common.Status.FAILURE
        else:
            print("Starting circuit")
            return py_trees.common.Status.SUCCESS

# Create the behavior tree using the py_tree library, uses recommended format with an extra waypoint 
# and added conditions for map creation and program termination

tree = Sequence("Main",children=[
            PositionArm("safe position", blackboard, 'safe_position'),
            Selector("Does map exist?",children=[
                doesMapExist("Test for map"),
                Parallel("Mapping",policy=py_trees.common.ParallelPolicy.SuccessOnOne(), children=[
                    Mapping("map the environment",blackboard),
                    Navigation("move around the table",blackboard)
                    ])
                ],memory=True),
                # Honey
                isCircuitFinished("Test for circuit", blackboard),
                Planning("Compute path to counter",blackboard, (1.16, 0.479)),
                Navigation("Move to counter", blackboard),
                CameraScan("honey", blackboard),
                TestIkpy("Calculate arm to honey", blackboard, 'honey_robot'),
                PositionArm("Move arm to honey", blackboard, 'new_position'),
                LowerRobot("Lower to honey", blackboard),
                CloseGripper("Close on honey", blackboard),
                RaiseRobot("Raise with honey", blackboard),
                PositionArm("Back to safe position", blackboard, 'safe_position'),
                Planning("Compute path to table", blackboard, (-0.038, -1.75)),
                Navigation("Move to table", blackboard),
                PositionArm("Place honey", blackboard, 'drop_position'),
                OpenGripper("Drop honey", blackboard),
                PositionArm("Back to safe position", blackboard, 'safe_position'),
                # Blue Jam
                Planning("Compute path to table edge",blackboard, (0.689, -0.364)),
                Navigation("Move to table edge", blackboard),
                Planning("Compute path to upper left",blackboard, (-1.64, 0.28)),
                Navigation("Move to upper left", blackboard),
                Planning("Compute path to counter",blackboard, (1.141, 0.479)),
                Navigation("Move to counter", blackboard),
                CameraScan("blue_jam", blackboard),
                TestIkpy("Calculate arm to blue_jam", blackboard, 'blue_jam_robot'),
                PositionArm("Move arm to blue_jam", blackboard, 'new_position'),
                MicroMovement("Move towards blue_jam", blackboard, 'forward', 80),
                CloseGripper("Close on blue_jam", blackboard),
                MicroMovement("Move away with blue_jam", blackboard, 'backward', 80),
                RaiseRobot("Lift blue_jam", blackboard),
                Planning("Compute path to table", blackboard, (-0.038, -1.5)),
                Navigation("Move to table", blackboard),
                OpenGripper("Drop blue_jam", blackboard),
                PositionArm("Back to safe position", blackboard, 'safe_position'),
                # Red Jam
                Planning("Compute path to table edge",blackboard, (0.689, -0.364)),
                Navigation("Move to table edge", blackboard),
                Planning("Compute path to central",blackboard, (0, 0)),
                Navigation("Cove to central", blackboard),
                Planning("Compute path to counter",blackboard, (1.141, -0.233)),
                Navigation("Move to counter", blackboard),
                CameraScan("red_jam", blackboard),
                TestIkpy("Calculate arm to red_jam", blackboard, 'red_jam_robot'),
                MicroMovement("Move away from red_jam", blackboard, 'backward', 80),
                PositionArm("Move arm to red_jam", blackboard, 'new_position'),
                # MicroMovement("Honey", blackboard, 'right', 15),
                MicroMovement("Move towards red_jam", blackboard, 'forward', 215),
                CloseGripper("Close blue_jam", blackboard),
                MicroMovement("Bring back red_jam", blackboard, 'backward', 80),
                Planning("Compute path to table", blackboard, (-0.038, -1.3)),
                Navigation("Move to table", blackboard),
                OpenGripper("Drop red_jam", blackboard),
                PositionArm("Back to safe last", blackboard, 'safe_position')
                ],memory=True)
             
             
tree.setup_with_descendants()




while robot.step(timestep) != -1 and blackboard.retrieve("circuit_finished") == False:
   tree.tick_once() 
   
    


