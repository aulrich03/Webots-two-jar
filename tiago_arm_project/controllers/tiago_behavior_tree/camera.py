import py_trees
from controller import Robot, Supervisor
from matplotlib import pyplot as plt
import numpy as np
from py_trees.composites import Sequence, Parallel, Selector
from scipy import signal

class CameraScan(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(CameraScan, self).__init__(name)
        self.blackboard = blackboard
        self.robot = blackboard.retrieve('robot')
        
        
    def setup(self):
        
        print("CAMERA")
        self.timestep = 32
        
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
        self.camera = self.robot.getDevice("Astra rgb")
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
        
        
        
    def initialise(self):
    
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
        
        self.objects = self.camera.getRecognitionObjects()
        
        print("objects: ", self.objects)
        
        torso_z = self.encoders["torso_lift_joint_sensor"].getValue()
        
        z = 0.6 + torso_z
        
        print("z: ", z)
        
        T_0_1 = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, z],
                          [0, 0, 0, 1]])
                          
                          
        theta = self.encoders["head_1_joint_sensor"].getValue()
        
        
        T_1_2 = np.array([[np.cos(theta), -np.sin(theta), 0, 0.182],
                          [np.sin(theta),  np.cos(theta), 0,   0  ],
                          [     0,              0,        1,   0  ],
                          [     0,              0,        0,   1  ]])
                          
                        
                        
        alpha = self.encoders["head_2_joint_sensor"].getValue()
        
        
        
        T_2_3 = np.array([[np.cos(alpha), 0, np.sin(alpha), 0.005],
                          [     0,        1,       0,         0  ],
                          [-np.sin(alpha),0, np.cos(alpha), 0.098],
                          [     0,        0,       0,         1  ]])
                          
                          
        T_3_4 =  np.array([[1, 0, 0, 0.107],
                          [0, 0, 1, 0.0802],
                          [0, -1, 0, 0],
                          [0, 0, 0, 1]])
                          
                          
        # T_4_5 =  np.array([[1, 0, 0, -0.028],
                          # [0, 1, 0, -0.035],
                          # [0, 0, 1, -0.009],
                          # [0, 0, 0, 1]])
                          
                          
        # T_5_6 =  np.array([[1, 0, 0, 0.027],
                          # [0, 1, 0, 0.011],
                          # [0, 0, 1, 0.034],
                          # [0, 0, 0, 1]])
                          
        #TRY TRANSFORMING TO BASE THEN TO ARM 1 JOINT
                          
        phi = 1.5708
        T_front_arm = np.array([[np.cos(phi), -np.sin(phi), 0, 0.026],
                         [np.sin(phi), np.cos(phi), 0, 0.14 ],
                         [0, 0, 1, -0.232],
                         [0, 0, 0, 1]])
               
        phi = -1.5708
        T_front_arm_link = np.array([[np.cos(phi), -np.sin(phi), 0, -0.037],
                         [np.sin(phi), np.cos(phi), 0, 0.0388],
                         [0, 0, 1, 0.0224],
                         [0, 0, 0, 1]])
                         
                         
                         
        phi = 0.649
        T_first_arm_joint = np.array([[np.cos(phi), -np.sin(phi), 0, 0.025],
                         [np.sin(phi), np.cos(phi), 0, 0.194],
                         [0, 0, 1, -0.16],
                         [0, 0, 0, 1]])
                         
        phi = 1.5708
        T_front_arm_joint = np.array([[np.cos(phi), -np.sin(phi), 0, 0],
                         [np.sin(phi), np.cos(phi), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])                           
                         
        
        T_arm_joint = T_front_arm
                         
        # phi2 = 0.649
        # T_base = np.array([[np.cos(phi2), -np.sin(phi2), 0, 0.025],
                         # [np.sin(phi2), np.cos(phi2), 0, 0.194 ],
                         # [0, 0, 1, -0.16],
                         # [0, 0, 0, 1]])
                          
        # self.T_0_3 = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4
        
        # self.T_0_arm = T_arm_joint @ T_0_1
        
        # if self.name == "locate_jars":
        
        
            # honey = np.array([*list(self.objects[0].getPosition()), 1])
            # red_jam = np.array([*list(self.objects[1].getPosition()), 1])
            # blue_jam = np.array([*list(self.objects[2].getPosition()), 1])
            
            # honey_robot = [-(honey[2])+0.22, -honey[0]+0.08, (honey[1])+0.97]
            # blue_jam_robot = self.T_0_6 @ blue_jam
            # red_jam_robot = self.T_0_6 @ red_jam
            
            
            # xw = self.gps.getValues()[0]
            # yw = self.gps.getValues()[1]
            
            # theta=np.arctan2(self.compass.getValues()[0],self.compass.getValues()[1])
            
            # w_T_r = np.array([[np.cos(theta), -np.sin(theta), xw],
                             # [np.sin(theta), np.cos(theta), yw],
                             # [0, 0, 1]])
                             
            
            # D_honey = w_T_r @ np.array([honey_robot[0], honey_robot[1], 1])
            # D_blue_jam = w_T_r @ np.array([blue_jam_robot[0], blue_jam_robot[1], 1])
            # D_red_jam = w_T_r @ np.array([red_jam_robot[0], red_jam_robot[1], 1])
            
            # self.blackboard.add('D_honey', [1.1, D_honey[1]])
            # self.blackboard.add('D_blue_jam', [1.3, D_blue_jam[1]])
            # self.blackboard.add('D_red_jam', [1.1, D_red_jam[1]])
            
            # print("Honey: ", self.blackboard.retrieve('D_honey'))
            
            
        if self.name.startswith('honey'):
            
            honey = np.array([*list(self.objects[0].getPosition()), 1])
            honey_robot = honey @ T_arm_joint
            
            honey_robot[2] -= 0.7
            honey_robot[1] += 0.1
            honey_robot[0] += 0.045
            
            
            self.blackboard.add('honey_robot', honey_robot[0:3])
            
        elif self.name.startswith('blue_jam'):
            
            blue_jam = np.array([*list(self.objects[0].getPosition()), 1])
            blue_jam_robot = T_arm_joint @ blue_jam
            
            blue_jam_robot[2] += 3.7
            blue_jam_robot[1] -= 0.03
            blue_jam_robot[0] -= 0.685
            
            self.blackboard.add('blue_jam_robot', blue_jam_robot[0:3])
            
            
        elif self.name.startswith('red_jam'):
            
            red_jam = np.array([*list(self.objects[0].getPosition()), 1])
            red_jam_robot = T_arm_joint @ red_jam
            
            red_jam_robot[2] += 3.3
            red_jam_robot[1] -= 0.03
            red_jam_robot[0] -= 0.615   
            
            self.blackboard.add('red_jam_robot', red_jam_robot[0:3])
            
        
        
    def update(self):
        
        
        
        
        return py_trees.common.Status.SUCCESS
        
        
    def terminate(self, new_status):
        print(self.name)
        