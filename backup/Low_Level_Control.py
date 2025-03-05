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
        self.serial_device = serial.Serial('/dev/ttyACM0', 921600, timeout=0.5)
        self.motor_control = MotorControl(self.serial_device)

        self.dm_motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
        self.motor_control.addMotor(self.dm_motor1)

        if self.motor_control.switchControlMode(self.dm_motor1, Control_Type.MIT):
            print("DM_CAN Motor1: switched to MIT control mode")

        self.motor_control.save_motor_param(self.dm_motor1)
        self.motor_control.enable(self.dm_motor1)

        self.Is_Run = False
        self.run_thread = None 

        with open(config_path, "r") as file:
            self.config = yaml.safe_load(file)

    def _run_motor(self):
        prev_time = time.time()
        count = 0
        """Function running the motor in the background."""
        while self.Is_Run:
            random_value = random.uniform(-3.5, 3.5)  # 產生 -3.5 到 3.5 之間的隨機數

            self.motor_control.controlMIT(self.dm_motor1, 10, 0.1, random_value, 0, 0)
            print("random-value",random_value)

            count += 1
            print("start",count)
            elapsed_time = time.time() - prev_time
            if elapsed_time >= 1.0:
                print(f"Motor control frequency: {count} Hz")
                count = 0
                prev_time = time.time()
            
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
        # self.motor_control.refresh_motor_status(self.dm_motor1)
        # time.sleep(1)
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
