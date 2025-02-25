import yaml
import serial
import time
import random
import traceback
import threading
from DM_CAN import *

class DualControlCmd:
    """Control two sets of DM_CAN motors using different serial ports."""
    def __init__(self, config_path="config/motor_config.yaml"):
        self.setup_serials()
        self.setup_motors()
        self.load_config(config_path)
        self.Is_Run_1 = False
        self.Is_Run_2 = False
        self.run_thread_1 = None
        self.run_thread_2 = None

    # [Previous setup_serials, setup_motors, load_config methods remain the same]
    def setup_serials(self):
        self.serial_device_1 = serial.Serial('/dev/ttyRedDogRight', 921600, timeout=0.5)
        self.motor_control_1 = MotorControl(self.serial_device_1)
        
        self.serial_device_2 = serial.Serial('/dev/ttyRedDogLeft', 921600, timeout=0.5)
        self.motor_control_2 = MotorControl(self.serial_device_2)

    def setup_motors(self):
        motor_names_1 = [1, 2, 3, 5, 6, 7]
        motor_params_1 = [(0x01, 0x11), (0x02, 0x12), (0x03, 0x13),
                         (0x05, 0x15), (0x06, 0x15), (0x07, 0x17)]

        motor_names_2 = [1, 2, 3, 5, 6, 7]
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
        print("motor_2",self.motors_2)

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

        # self.leg_motor_list = [
        #     [self.motors_1['FR_lower'], self.motors['FL_lower'], self.motors_1['RR_lower'], self.motors['RL_lower']],
        #     [self.motors_1['FR_higher'], self.motors['FL_higher'], self.motors_1['RR_higher'], self.motors['RL_higher']],
        #     [self.motors_1['FR_hip'], self.motors['FL_hip'], self.motors_1['RR_hip'], self.motors['RL_hip']]
        # ]


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

    # [Previous _run_motor_1, _run_motor_2, run, stop, reset, read methods remain the same]
    def _run_motor_1(self):
        while self.Is_Run_1:
            position = random.uniform(-3.5, 3.5)
            print(f"Set 1: Attempting to move to position: {position}")
            for motor_id, motor in self.motors_1.items():
                limits = self.motor_limits_1[motor_id]
                if position < limits["min_position"] or position > limits["max_position"]:
                    print(f"⚠️ Motor {motor_id} (Set 1) position out of range, stopping operation")
                    if limits["stop_on_exceed"]:
                        self.Is_Run_1 = False
                        return
                self.motor_control_1.controlMIT(motor, 50, 1, position, 0, 0)
            time.sleep(5)

    def _run_motor_2(self):
        while self.Is_Run_2:
            position = random.uniform(-3.5, 3.5)
            print(f"Set 2: Attempting to move to position: {position}")
            for motor_id, motor in self.motors_2.items():
                limits = self.motor_limits_2[motor_id]
                if position < limits["min_position"] or position > limits["max_position"]:
                    print(f"⚠️ Motor {motor_id} (Set 2) position out of range, stopping operation")
                    if limits["stop_on_exceed"]:
                        self.Is_Run_2 = False
                        return
                self.motor_control_2.controlMIT(motor, 50, 1, position, 0, 0)
            time.sleep(5)

    def run(self, motor_set=None):
        if motor_set in [None, 'set1'] and not self.Is_Run_1:
            self.Is_Run_1 = True
            for motor in self.motors_1.values():
                self.motor_control_1.enable(motor)
            self.run_thread_1 = threading.Thread(target=self._run_motor_1)
            self.run_thread_1.start()
            print("Motors Set 1 started running...")

        if motor_set in [None, 'set2'] and not self.Is_Run_2:
            self.Is_Run_2 = True
            for motor in self.motors_2.values():
                self.motor_control_2.enable(motor)
            self.run_thread_2 = threading.Thread(target=self._run_motor_2)
            self.run_thread_2.start()
            print("Motors Set 2 started running...")

    def stop(self, motor_set=None):
        if motor_set in [None, 'set1']:
            self.Is_Run_1 = False
            if self.run_thread_1 and self.run_thread_1.is_alive():
                self.run_thread_1.join()
            for motor in self.motors_1.values():
                self.motor_control_1.disable(motor)
            print("Motors Set 1 stopped running")

        if motor_set in [None, 'set2']:
            self.Is_Run_2 = False
            if self.run_thread_2 and self.run_thread_2.is_alive():
                self.run_thread_2.join()
            for motor in self.motors_2.values():
                self.motor_control_2.disable(motor)
            print("Motors Set 2 stopped running")

    def reset(self, motor_set=None):
        if motor_set in [None, 'set1']:
            for motor in self.motors_1.values():
                self.motor_control_1.controlMIT(motor, 50, 1, 0, 0, 0)
            print("Motors Set 1 reset")

        if motor_set in [None, 'set2']:
            for motor in self.motors_2.values():
                self.motor_control_2.controlMIT(motor, 50, 1, 0, 0, 0)
            print("Motors Set 2 reset")

    def read(self, motor_set=None):
        if motor_set in [None, 'set1']:
            print("=== Motors Set 1 Status ===")
            for motor in self.motors_1.values():
                self.motor_control_1.refresh_motor_status(motor)
                time.sleep(1)
                print(f"Motor {motor.SlaveID}: POS: {motor.getPosition()}, VEL: {motor.getVelocity()}, TORQUE: {motor.getTorque()}")

        if motor_set in [None, 'set2']:
            print("=== Motors Set 2 Status ===")
            for motor in self.motors_2.values():
                self.motor_control_2.refresh_motor_status(motor)
                time.sleep(1)
                print(f"Motor {motor.SlaveID}: POS: {motor.getPosition()}, VEL: {motor.getVelocity()}, TORQUE: {motor.getTorque()}")

    # New enable/disable methods
    def enable_motor(self, motor_set=None):
        """Enable motors. Can specify 'set1', 'set2', or None for both."""
        if motor_set in [None, 'set1']:
            for motor in self.motors_1.values():
                self.motor_control_1.enable(motor)
            print("Motors Set 1 enabled")

        if motor_set in [None, 'set2']:
            for motor in self.motors_2.values():
                self.motor_control_2.enable(motor)
            print("Motors Set 2 enabled")

    def disable_motor(self, motor_set=None):
        """Disable motors. Can specify 'set1', 'set2', or None for both."""
        if motor_set in [None, 'set1']:
            for motor in self.motors_1.values():
                self.motor_control_1.disable(motor)
            print("Motors Set 1 disabled")

        if motor_set in [None, 'set2']:
            for motor in self.motors_2.values():
                self.motor_control_2.disable(motor)
            print("Motors Set 2 disabled")

    def set_zero(self, motor_set=None):
        if motor_set in [None, 'set1']:
            for motor in self.motors_1.values():
                self.motor_control_1.set_zero_position(motor)
            print("Motors Set 1 set to zero")

        if motor_set in [None, 'set2']:
            for motor in self.motors_2.values():
                self.motor_control_2.set_zero_position(motor)
            print("Motors Set 2 set to zero")

        """Set the zero position."""
        
        print("Motor zero position set")

    def closeSystem(self):
        """Shut down the system."""
        self.stop()
        self.serial_device_1.close()
        self.serial_device_2.close()
        print("System closed")


def main():
    robot_control = DualControlCmd()
    
    command_dict = {
        "r": lambda: robot_control.run(),
        "r1": lambda: robot_control.run('set1'),
        "r2": lambda: robot_control.run('set2'),
        "stop": lambda: robot_control.stop(),
        "stop1": lambda: robot_control.stop('set1'),
        "stop2": lambda: robot_control.stop('set2'),
        "reset": lambda: robot_control.reset(),
        "reset1": lambda: robot_control.reset('set1'),
        "reset2": lambda: robot_control.reset('set2'),
        "read": lambda: robot_control.read(),
        "read1": lambda: robot_control.read('set1'),
        "read2": lambda: robot_control.read('set2'),
        "enable": lambda: robot_control.enable_motor(),
        "enable1": lambda: robot_control.enable_motor('set1'),
        "enable2": lambda: robot_control.enable_motor('set2'),
        "disable": lambda: robot_control.disable_motor(),
        "disable1": lambda: robot_control.disable_motor('set1'),
        "disable2": lambda: robot_control.disable_motor('set2'),
        "set" : lambda: robot_control.set_zero(),
    }
    
    print("Available commands:")
    print("r/r1/r2 - Run all motors/set1/set2")
    print("stop/stop1/stop2 - Stop all motors/set1/set2")
    print("reset/reset1/reset2 - Reset all motors/set1/set2")
    print("read/read1/read2 - Read status of all motors/set1/set2")
    print("enable/enable1/enable2 - Enable all motors/set1/set2")
    print("disable/disable1/disable2 - Disable all motors/set1/set2")
    print("exit - Close the system")
    
    while True:
        try:
            cmd = input("CMD: ")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                robot_control.closeSystem()
                break
            else:
                print("Invalid command")
        except Exception as e:
            traceback.print_exc()
            break

if __name__ == '__main__':
    main()