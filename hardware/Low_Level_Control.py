import yaml
import serial
import time
import math
import random
import traceback
import threading
from DM_CAN import *

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('motor_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        self.get_logger().info('Subscribed to joint state topic.')
        self.joint_data = {}

    def joint_state_callback(self, msg):
        self.joint_data = dict(zip(msg.name, msg.position))
        self.kp_kd_value = msg.effort
    
    def get_jointAngle_data(self):
        return self.joint_data, self.kp_kd_value
        
class MotorManager:
    def __init__(self, config_path="config/motor_config.yaml"):
        self.jointAngleSub = JointStateSubscriber()

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.jointAngleSub)

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.control_cmd = DualControlCmd()
        self.lock = threading.Lock()

        self.Is_Run_1 = False
        self.Is_Run_2 = False
        self.run_thread_1 = None
        self.run_thread_2 = None

        self.jointAngle_data = {}
        self.joint_angles = None
        self.kp_kd_list = None

        with open(config_path, "r") as file:
            self.motor_limits = yaml.safe_load(file)["motor_limits"]
    
    def stop_executor(self):
        self.executor.shutdown()
        self.spin_thread.join()  

    def is_within_limits(self, joint_name, angle):
        for motor_group in self.motor_limits.values():
            if joint_name in motor_group:
                lower, upper = motor_group[joint_name]
                return lower <= angle <= upper
        return False  
    
    def get_jointAngle_data(self):
        self.jointAngle_data, self.kp_kd_list = self.jointAngleSub.get_jointAngle_data()

        required_keys = ['frh', 'fru', 'frd', 'flh', 'flu', 'fld', 'rrh', 'rru', 'rrd', 'rlh', 'rlu', 'rld']
        for key in required_keys:
            if key not in self.jointAngle_data:
                print(f"Joint key '{key}' not found in joint states! Setting default value.")
                self.jointAngle_data[key] = 0.0  # Default safe value
            elif not self.is_within_limits(key, self.jointAngle_data[key]):
                print(f"Warning: {key} angle {self.jointAngle_data[key]} is out of bounds!")
                self.jointAngle_data[key] = max(min(self.jointAngle_data[key], self.motor_limits[key][1]), self.motor_limits[key][0])

        self.joint_angles_1 = np.array([[self.jointAngle_data['frd'],  self.jointAngle_data['rrd']],
                                      [self.jointAngle_data['fru'],  self.jointAngle_data['rru']],
                                      [self.jointAngle_data['frh'],  self.jointAngle_data['rrh']]])
        
        self.joint_angles_2 = np.array([[self.jointAngle_data['fld'],  self.jointAngle_data['rld']],
                                      [self.jointAngle_data['flu'],  self.jointAngle_data['rlu']],
                                      [self.jointAngle_data['flh'],  self.jointAngle_data['rlh']]])

    # [Previous _run_motor_1, _run_motor_2, run, stop, reset, read methods remain the same]
    def _run_motor_1(self):
        while self.Is_Run_1:
            self.get_jointAngle_data()
            self.control_cmd.motor_position_control_1(self.joint_angles_1, self.kp_kd_list)
            time.sleep(0.002)

    def _run_motor_2(self):
        while self.Is_Run_2:
            self.control_cmd.motor_position_control_2(self.joint_angles_2, self.kp_kd_list)
            time.sleep(0.002)

    def run(self):
        threads = []
        if not self.Is_Run_1:
            self.Is_Run_1 = True
            for motor in self.control_cmd.motors_1.values():
                self.control_cmd.motor_control_1.enable(motor)
            self.run_thread_1 = threading.Thread(target=self._run_motor_1)
            threads.append(self.run_thread_1)
            print("Motors Set 1 started running...")

        if not self.Is_Run_2:
            self.Is_Run_2 = True
            for motor in self.control_cmd.motors_2.values():
                self.control_cmd.motor_control_2.enable(motor)
            self.run_thread_2 = threading.Thread(target=self._run_motor_2)
            threads.append(self.run_thread_2)
            print("Motors Set 2 started running...")
        
        for t in threads:
            t.start()
    
    def stop(self):
        self.executor.shutdown()
        self.spin_thread.join() 

        self.Is_Run_1 = False
        if self.run_thread_1 and self.run_thread_1.is_alive():
            self.run_thread_1.join()
        print("Motors Set 1 stopped running")

        self.Is_Run_2 = False
        if self.run_thread_2 and self.run_thread_2.is_alive():
            self.run_thread_2.join()
        print("Motors Set 2 stopped running")

class DualControlCmd:
    """Control two sets of DM_CAN motors using different serial ports."""
    def __init__(self, config_path="config/motor_config.yaml"):
        self.setup_serials()
        self.setup_motors()
        self.load_config(config_path)
        self.lock = threading.Lock()

    # [Previous setup_serials, setup_motors, load_config methods remain the same]
    def setup_serials(self):
        self.serial_device_1 = serial.Serial('/dev/ttyRedDogRight', 921600, timeout=0.5)
        self.motor_control_1 = MotorControl(self.serial_device_1)
        
        self.serial_device_2 = serial.Serial('/dev/ttyRedDogLeft', 921600, timeout=0.5)
        self.motor_control_2 = MotorControl(self.serial_device_2)

    def setup_motors(self):
        motor_names_1 = [ 'FR_hip', 'FR_higher', 'FR_lower', 'RR_hip', 'RR_higher', 'RR_lower']
        motor_params_1 = [(0x01, 0x11), (0x02, 0x12), (0x03, 0x13),
                         (0x05, 0x15), (0x06, 0x16), (0x07, 0x17)]

        motor_names_2 = [ 'FL_hip', 'FL_higher', 'FL_lower', 'RL_hip', 'RL_higher', 'RL_lower']
        motor_params_2 = [(0x01, 0x11), (0x02, 0x12), (0x03, 0x13),
                         (0x05, 0x15), (0x06, 0x16), (0x07, 0x17)]

        self.motors_1 = {
            name: Motor(DM_Motor_Type.DM4310, id_1, id_2)
            for name, (id_1, id_2) in zip(motor_names_1, motor_params_1)
        }
        
        self.motors_2 = {
            name: Motor(DM_Motor_Type.DM4310, id_1, id_2)
            for name, (id_1, id_2) in zip(motor_names_2, motor_params_2)
        }

        for motor in self.motors_1.values():
            self.motor_control_1.addMotor(motor)
            if self.motor_control_1.switchControlMode(motor, Control_Type.MIT):
                print(f"DM_CAN Motor {motor.SlaveID} (Set 1): switched to MIT control mode")
            self.motor_control_1.save_motor_param(motor)
            self.motor_control_1.enable(motor)

        for motor in self.motors_2.values():
            self.motor_control_2.addMotor(motor)
            if self.motor_control_2.switchControlMode(motor, Control_Type.MIT):
                print(f"DM_CAN Motor {motor.SlaveID} (Set 2): switched to MIT control mode")
            self.motor_control_2.save_motor_param(motor)
            self.motor_control_2.enable(motor)

        self.leg_motor_list_1 = [
                        [self.motors_1['FR_lower'],   self.motors_1['RR_lower']  ],
                        [self.motors_1['FR_higher'],  self.motors_1['RR_higher'] ],
                        [self.motors_1['FR_hip'],     self.motors_1['RR_hip'],   ]
        ]

        self.leg_motor_list_2 = [
                        [self.motors_2['FL_lower'],   self.motors_2['RL_lower']  ],
                        [self.motors_2['FL_higher'],  self.motors_2['RL_higher'] ],
                        [self.motors_2['FL_hip'],     self.motors_2['RL_hip']    ]
        ]

    def load_config(self, config_path):
        with open(config_path, "r") as file:
            self.config = yaml.safe_load(file)
        
        self.motor_limits_1 = {}
        self.motor_limits_2 = {}
        
        for motor_id in self.motors_1:
            limits = self.config.get("motor_limits", {}).get(f"motor{motor_id}", {})
            self.motor_limits_1[motor_id] = {
                "min_position": limits.get("min_position", -2),
                "max_position": limits.get("max_position", 2),
                "stop_on_exceed": limits.get("stop_on_exceed", True)
            }
            
        for motor_id in self.motors_2:
            limits = self.config.get("motor_limits", {}).get(f"motor{motor_id}", {})
            self.motor_limits_2[motor_id] = {
                "min_position": limits.get("min_position", -2),
                "max_position": limits.get("max_position", 2),
                "stop_on_exceed": limits.get("stop_on_exceed", True)
            }

    def reset(self, motor_set=None):
        threads = []
        if motor_set in [None, 'set1']:
            t1 = threading.Thread(target=self.motor_position_control_1)
            threads.append(t1)
            print("Motors Set 1 reset")

        if motor_set in [None, 'set2']:
            t2 = threading.Thread(target=self.motor_position_control_2)
            threads.append(t2)
            print("Motors Set 2 reset")

        for t in threads:
            t.start()

        for t in threads:
            t.join()
    
    def motor_position_control_1(self, position=None, kp_kd_list=None):
        with self.lock:
            if position is None:
                position = [[     3 ,    -3 ], 
                            [  -1.6 ,   1.6 ], 
                            [     0 ,     0 ]] 
                # position = [[     0 ,     0], 
                #             [     0 ,     0], 
                #             [     0 ,     0]]
            
            if kp_kd_list is None:
                kp_kd_list = [ 3, 0.1]
                
            for i, motor_list in enumerate(self.leg_motor_list_1):
                for j, motor in enumerate(motor_list):
                    self.motor_control_1.controlMIT(motor, kp_kd_list[0], kp_kd_list[1], position[i][j], 0, 0)
        
    def motor_position_control_2(self, position=None, kp_kd_list=None):
        with self.lock:  
            if position is None:
                position = [[    -3 ,     3], 
                            [   1.6 ,  -1.6], 
                            [     0 ,     0]] 
                # position = [[     0 ,     0], 
                #             [     0 ,     0], 
                #             [     0 ,     0]] 

            if kp_kd_list is None:
                kp_kd_list = [ 3, 0.1]

            for i, motor_list in enumerate(self.leg_motor_list_2):
                for j, motor in enumerate(motor_list):
                    self.motor_control_2.controlMIT(motor, kp_kd_list[0], kp_kd_list[1], position[i][j], 0, 0)

    def read(self):
        self.update_joint_state()
        np.set_printoptions(suppress=True)  # 禁用科學記號
        # print(self.joint_positions / math.pi * 180 )
        print(self.joint_positions)

    def update_joint_state(self):
        for motor in self.motors_1.values():
            self.motor_control_1.refresh_motor_status(motor)

        for motor in self.motors_2.values():
            self.motor_control_2.refresh_motor_status(motor)

        self.joint_positions = np.zeros((3, 4))

        for i, (motor_list_1, motor_list_2) in enumerate(zip(self.leg_motor_list_1, self.leg_motor_list_2)):
            self.joint_positions[i] = [
                motor_list_1[0].getPosition() , motor_list_2[0].getPosition() ,
                motor_list_1[1].getPosition() , motor_list_2[1].getPosition() 
            ]

    # New enable/disable methods
    def enable_motor(self):
        """Enable motors. Can specify 'set1', 'set2', or None for both."""
        for motor in self.motors_1.values():
            self.motor_control_1.enable(motor)
        print("Motors Set 1 enabled")

        for motor in self.motors_2.values():
            self.motor_control_2.enable(motor)
        print("Motors Set 2 enabled")

    def disable_motor(self):
        """Disable motors. Can specify 'set1', 'set2', or None for both."""
        for motor in self.motors_1.values():
            self.motor_control_1.disable(motor)
        print("Motors Set 1 disabled")

        for motor in self.motors_2.values():
            self.motor_control_2.disable(motor)
        print("Motors Set 2 disabled")

    def set_zero(self):
        for motor in self.motors_1.values():
            self.motor_control_1.set_zero_position(motor)

        for motor in self.motors_2.values():
            self.motor_control_2.set_zero_position(motor)

        """Set the zero position."""
        print("Motor zero position set")

    def closeSystem(self):
        """Shut down the system."""
        self.serial_device_1.close()
        self.serial_device_2.close()
        print("System closed")


def main():
    rclpy.init()
    motor_manager = MotorManager()
    
    command_dict = {
        "r": motor_manager.run,
        "stop": motor_manager.stop,
        "reset": motor_manager.control_cmd.reset,
        "read": motor_manager.control_cmd.read,
        "enable": motor_manager.control_cmd.enable_motor,
        "disable": motor_manager.control_cmd.disable_motor,
        "set" : motor_manager.control_cmd.set_zero,
    }
    
    print("Available commands:")
    print("r - Run all motors")
    print("stop - Stop all motors")
    print("reset - Reset all motors")
    print("read - Read status of all motors")
    print("enable - Enable all motors")
    print("disable - Disable all motors")
    print("exit - Close the system")
    
    while True:
        try:
            cmd = input("CMD: ")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                motor_manager.stop()
                motor_manager.control_cmd.closeSystem()
                break
            else:
                print("Invalid command")
        except Exception as e:
            traceback.print_exc()
            break

if __name__ == '__main__':
    main()