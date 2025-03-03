import yaml
import serial
import time
import random
import traceback
import threading
from DM_CAN import *

class ControlCmd:
    """Control DM_CAN motors."""
    
    def __init__(self, config_path="motor_config.yaml"):
        self.serial_device = serial.Serial('/dev/ttyRedDogRight', 921600, timeout=0.5)
        self.motor_control = MotorControl(self.serial_device)

        self.dm_motor1 = Motor(DM_Motor_Type.DM4310, 0x07, 0x17)
        # self.dm_motor2 = Motor(DM_Motor_Type.DM4310, 0x06, 0x16)
        # self.dm_motor3 = Motor(DM_Motor_Type.DM4310, 0x07, 0x17)
        
        self.motor_control.addMotor(self.dm_motor1)
        # self.motor_control.addMotor(self.dm_motor2)
        # self.motor_control.addMotor(self.dm_motor3)

        if self.motor_control.switchControlMode(self.dm_motor1, Control_Type.MIT):
            print("DM_CAN Motor1: switched to MIT control mode")
        
        # if self.motor_control.switchControlMode(self.dm_motor2, Control_Type.MIT):
        #     print("DM_CAN Motor2: switched to MIT control mode")
        
        # if self.motor_control.switchControlMode(self.dm_motor3, Control_Type.MIT):
        #     print("DM_CAN Motor3: switched to MIT control mode")

        self.motor_control.save_motor_param(self.dm_motor1)
        self.motor_control.enable(self.dm_motor1)

        # self.motor_control.save_motor_param(self.dm_motor2)
        # self.motor_control.enable(self.dm_motor2)

        # self.motor_control.save_motor_param(self.dm_motor3)
        # self.motor_control.enable(self.dm_motor3)

        self.Is_Run = False
        self.run_thread = None 

        with open(config_path, "r") as file:
            self.config = yaml.safe_load(file)
        
        motor_limits_1 = self.config.get("motor_limits", {}).get("motor1", {})
        self.min_position_1 = motor_limits_1.get("min_position", -2)
        self.max_position_1 = motor_limits_1.get("max_position", 2)
        self.stop_on_exceed_1 = motor_limits_1.get("stop_on_exceed", True)

        motor_limits_2 = self.config.get("motor_limits", {}).get("motor2", {})
        self.min_position_2 = motor_limits_2.get("min_position", -2)
        self.max_position_2 = motor_limits_2.get("max_position", 2)
        self.stop_on_exceed_2 = motor_limits_2.get("stop_on_exceed", True)

        motor_limits_3 = self.config.get("motor_limits", {}).get("motor3", {})
        self.min_position_3 = motor_limits_3.get("min_position", -2)
        self.max_position_3 = motor_limits_3.get("max_position", 2)
        self.stop_on_exceed_3 = motor_limits_3.get("stop_on_exceed", True)

    def _run_motor(self):
        """Function running the motor in the background."""
        while self.Is_Run:
            position = random.uniform(-3.5, 3.5)  # Simulate random positions, possibly exceeding limits
            print(f"Attempting to move to position: {position}")
            self.current_position = position

            # **Check if the position exceeds limits**
            if position < self.min_position or position > self.max_position:
                print(f"⚠️ Position out of range ({self.min_position}, {self.max_position}), stopping operation")
                if self.stop_on_exceed:
                    self.Is_Run = False  # Set to False, allowing external stop() to handle it
                    return  # **Allow thread to exit naturally without calling self.stop()**

            self.motor_control.controlMIT(self.dm_motor1, 50, 1, position, 0, 0)
            time.sleep(5)

    def run(self):
        """Start running the motor using a thread to avoid blocking the main program."""
        if not self.Is_Run:
            self.Is_Run = True
            self.motor_control.enable(self.dm_motor1)
            self.run_thread = threading.Thread(target=self._run_motor)
            self.run_thread.start()
            print("Motor started running...")

    def stop(self):
        """Stop the motor operation."""
        self.Is_Run = False
        if self.run_thread and self.run_thread.is_alive():
            self.run_thread.join()  # **Ensure run_thread is alive before joining**
        self.motor_control.disable(self.dm_motor1)
        print("Motor stopped running")

    def reset(self):
        """Reset the motor."""
        self.motor_control.disable(self.dm_motor1)
        self.motor_control.enable(self.dm_motor1)
        self.motor_control.controlMIT(self.dm_motor1, 10, 1, 0, 0, 0)
        print("Motor reset")

    def read(self):
        # self.serial_device.reset_input_buffer()
        self.motor_control.refresh_motor_status(self.dm_motor1)
        time.sleep(1)
        print("Motor1:", "POS:", self.dm_motor1.getPosition(), "VEL:", self.dm_motor1.getVelocity(), "TORQUE:", self.dm_motor1.getTorque())

    def disable_motor(self):
        """Disable the motor."""
        self.motor_control.disable(self.dm_motor1)
        self.motor_control.enable(self.dm_motor1)
        print("Motor disabled")

    def enable_motor(self):
        """Enable the motor."""
        self.motor_control.disable(self.dm_motor1)
        print("Motor enabled")

    def set_zero(self):
        """Set the zero position."""
        self.motor_control.set_zero_position(self.dm_motor1)
        print("Motor zero position set")

    def closeSystem(self):
        """Shut down the system."""
        self.stop()
        self.serial_device.close()
        print("System closed")


def main():
    robot_control = ControlCmd()

    command_dict = {
        "r": robot_control.run,
        "stop": robot_control.stop,
        "reset": robot_control.reset,
        "read": robot_control.read,
        "disable": robot_control.disable_motor,
        "enable": robot_control.enable_motor,
        "setZero": robot_control.set_zero,
    }

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
