import numpy as np
import py_trees
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink






class TestIkpy(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, arm_position):
        super(TestIkpy, self).__init__(name) 
        self.blackboard = blackboard
        self.robot = blackboard.retrieve('robot')
        self.arm_position = arm_position
        
    def setup(self):
        
        # with open("tiago_urdf.urdf", "w") as file:  
            # file.write(self.robot.getUrdf())
        # self.urdf = r"C:\Users\alexa\Downloads\tiago_arm_project\protos\Robot.urdf"
        self.urdf = r"..\..\protos\Robot.urdf"
        self.timestep = 32
        
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')
    
    
    def initialise(self):
        
        # NEW IKPY CODE
        
        self.arm_position = self.blackboard.retrieve(self.arm_position)
        
        base_elements=["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_joint", "TIAGo front arm_3"]
        
        
        my_chain = Chain.from_urdf_file(self.urdf,
        last_link_vector=[0.004, 0, -0.1741],
        base_elements=['TIAGo front arm'],
        active_links_mask=[False, True, True, True, True, True,
                      True, False, False, False, False, False])
       
        
        encoders = {
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
        
        for jname in encoders:
            encoders[jname] = self.robot.getDevice(jname)
            encoders[jname].enable(self.timestep)
            
        initial_position = []
        
        for encoder in list(encoders.keys())[1:8]:
            initial_position.append(encoders[encoder].getValue())
            
          
        initial_position = [0.1] + initial_position + [0,0,0,0]  
        
        
        
        target_orientation = [0,0,1]
 
        ikResults = my_chain.inverse_kinematics(self.arm_position, initial_position=initial_position) 
            
        
        
        
        ikForward = my_chain.forward_kinematics(ikResults)
        
       
        self.joints = [
       'torso_lift_joint','arm_1_joint','arm_2_joint','arm_3_joint',
       'arm_4_joint','arm_5_joint','arm_6_joint','arm_7_joint',
       'gripper_left_finger_joint','gripper_right_finger_joint',
       'head_1_joint','head_2_joint']
        
        self.new_position = {}
        
        index = 0
        
        for joint in self.joints:
            self.new_position[joint] = ikResults[index]
            index += 1
            
        self.new_position['gripper_left_finger_joint'] = 0.045
        self.new_position['gripper_right_finger_joint'] = 0.045
        # self.new_position['arm_7_joint'] = np.pi / 6
            
        
      
        
    def update(self):
    
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        return py_trees.common.Status.SUCCESS    
    
    
    def terminate(self, new_status):
        
        self.blackboard.add('new_position', self.new_position)
        print(self.name)