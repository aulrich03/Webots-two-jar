import py_trees
import numpy as np


class PositionArm(py_trees.behaviour.Behaviour):
    
    def __init__(self, name, blackboard, goal_position):
        super(PositionArm, self).__init__(name)
        self.blackboard = blackboard
        self.robot = blackboard.retrieve('robot')
        self.goal_position = goal_position
        
        
    def setup(self):
        
        self.reachedPosition = False
        
        self.timestep = 32
       
        self.encoders = {
       'torso_lift_joint_sensor' : 0,
       'arm_1_joint_sensor' : 0,
       'arm_2_joint_sensor' : 0,
       'arm_3_joint_sensor' : 0,
       'arm_4_joint_sensor' : 0,
       'arm_5_joint_sensor' : 0,
       'arm_6_joint_sensor' : 0,
       'arm_7_joint_sensor' : 0,
       'gripper_left_sensor_finger_joint' : 0,
       'gripper_right_sensor_finger_joint' : 0,
       'head_1_joint_sensor' : 0,
       'head_2_joint_sensor' : 0}
        
        
        for jname in self.encoders:
            self.encoders[jname] = self.robot.getDevice(jname)
            self.encoders[jname].enable(self.timestep)
            
            
        self.motor_left = self.robot.getDevice('wheel_left_joint')
        self.motor_right = self.robot.getDevice('wheel_right_joint')
        
        self.motor_left.setPosition(float('inf'))
        self.motor_right.setPosition(float('inf'))
        
        
        
    def initialise(self):
        
        self.goal_position = self.blackboard.retrieve(self.goal_position)
        
        if self.name == "Move arm to blue_jam" or self.name == "Move arm to red_jam":
            self.goal_position["arm_6_joint"] += 0.28
            self.goal_position["arm_7_joint"] -= 0.1
        
        self.motor_left.setVelocity(0)
        self.motor_right.setVelocity(0)
        
            
        self.error = float('inf')
            
            
    def update(self):
                    
       self.reachedPosition = self.error <= 0.0005
       
           
       encoder_vals = []
       
       for jname in self.encoders:
           encoder_vals.append(self.encoders[jname].getValue())    
       
       encoder_vals = np.array(encoder_vals)[0:8]
       
       self.error = np.sum((encoder_vals -
                    np.array(list(self.goal_position.values()))[0:8])**2)
                    
                    
       for key in self.goal_position:
           self.robot.getDevice(key).setPosition(self.goal_position[key])
            
       
       if self.reachedPosition == True:
           return py_trees.common.Status.SUCCESS
       else:
           return py_trees.common.Status.RUNNING
            
            
            
    def terminate(self, new_status):
        
       print("Location reached")
            
        