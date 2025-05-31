from copy import deepcopy
import enum
from lerobot.common.robot_devices.utils import \
    RobotDeviceAlreadyConnectedError, \
    RobotDeviceNotConnectedError
from xarm.wrapper import XArmAPI
import numpy as np
import threading
import time
import logging


class TorqueMode(enum.Enum):
    ENABLED = 1
    DISABLED = 0

class xArmWrapper:
    """Wrapper for the xArm Python SDK"""

    def __init__(
        self,
        port: str,
        motors: dict[str, tuple[int, str]],
        mock=False,
    ):
        print("Initializing xArmWrapper")  # Debug print
        self.port = port
        self.motors = motors
        self.mock = mock

        self.calibration = None
        self.is_connected = False
        self.logs = {}

        self.api = None

        self.MAX_SPEED_LIMIT = None
        self.MAX_ACC_LIMIT = None

        # Stop event
        self.stop_event = threading.Event()
        self.replay_mode = False

        # Create and start the digital input monitoring thread
        print("Creating monitor thread")  # Debug print
        self.monitor_input_thread = threading.Thread(target=self.monitor_digital_input, args=(self.stop_event,))


    @property
    def motor_names(self) -> list[str]:
        return list(self.motors.keys())

    @property
    def motor_models(self) -> list[str]:
        return [model for _, model in self.motors.values()]

    @property
    def motor_indices(self) -> list[int]:
        return [idx for idx, _ in self.motors.values()]

    def connect(self):
        print("Connecting to xArm")  # Debug print
        print(self.port)
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                f"DynamixelMotorsBus({self.port}) is already connected. Do not call `motors_bus.connect()` twice."
            )

        if self.mock:
            print("Mock mode, not connecting to real device")  # Debug print
            return
        else:
            self.api = XArmAPI(self.port)

        try:
            if not self.api.connected:
                raise OSError(f"Failed to connect to xArm API @ '{self.port}'.")
            print("Successfully connected to xArm")  # Debug print
        except Exception as e:
            print(f"Exception while connecting in xArmWrapper: {e}")
            raise

        # Allow to read and write
        self.is_connected = True

        # Start the monitoring thread after successful connection
        self.monitor_input_thread.start()
        print("Monitor thread started")  # Debug print

    def write(self, data_name, values: int | float | np.ndarray, motor_names: str | list[str] | None = None):
        pass  # TODO (@vmayoral): implement if of interest

    def read(self, data_name, motor_names: str | list[str] | None = None):
        pass  # TODO (@vmayoral): implement if of interest

    def enable(self, follower: bool = False,return_init_pos: bool = False,init_pos: np.ndarray = None):
        self.api.motion_enable(enable=True)
        self.api.clean_error()
        self.api.set_mode(0)
        self.api.set_state(0)
        time.sleep(2)
        if init_pos is None:
            _, init_pos = tuple(self.api.get_initial_point())
        self.api.set_servo_angle(angle=init_pos,wait=True,is_radian=False)
        if follower:
            self.api.set_mode(1)
        else:
            self.api.set_mode(0)
        self.api.set_state(state=0)
        #get the indixof the gripper
        if self.motor_models[-1] == "gipper":
            self.api.set_gripper_mode(0)
            self.api.set_gripper_enable(True)
            self.api.set_gripper_speed(5000)  # default speed, as there's no way to fetch gripper speed from API
        elif self.motor_models[-1] == "robotiq":
            #self.api.set_collision_tool_model(5)
            self.api.robotiq_reset()
            self.api.robotiq_set_activate()
            self.api.set_gripper_speed(5000)
        else:
            raise ValueError(f"Unsupported gripper model: {self.motor_models[-1]}")

        # # Initialize the global speed and acceleration limits
        # self.initialize_limits()  # not acting as expected

        # assume leader by default
        if not follower:
            self.api.set_mode(2)
            self.api.set_state(0)
            # Light up the digital output 2 (button), to signal manual mode
            self.api.set_tgpio_digital(ionum=2, value=1)
        
        if return_init_pos:
            # Return the initial position of the robot
            return np.array(init_pos)
        

    def disconnect(self):
        print("Disconnecting from xArm")  # Debug print
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"FeetechMotorsBus({self.port}) is not connected. Try running `motors_bus.connect()` first."
            )

        # Turn off manual mode after recording
        self.api.set_mode(0)
        self.api.set_state(0)
        # Light down the digital output 2 (button), to signal manual mode
        self.api.set_tgpio_digital(ionum=2, value=0)
        # Disconnect both arms
        self.api.disconnect()

        # Stop events and threads
        self.stop_event.set()
        print("Waiting for monitor thread to join")  # Debug print
        self.monitor_input_thread.join()
        print("Monitor thread joined")  # Debug print

        # Signal as disconnected
        self.is_connected = False

    def __del__(self):
        if getattr(self, "is_connected", False):
            self.disconnect()

    def initialize_limits(self):
        # heuristic: 1/3 of the max speed and acceleration limits
        #  for testing purposes
        self.MAX_SPEED_LIMIT = max(self.api.joint_speed_limit)/3
        self.MAX_ACC_LIMIT = max(self.api.joint_acc_limit)/3

    def get_position(self):
        if self.motor_models[-1] == "gipper":
            code, angles = self.api.get_servo_angle()
            code_gripper, pos_gripper = self.api.get_gripper_position()
            pos = angles[:-1] + [pos_gripper]  # discard 7th dof, which is not present in U850
            #pos = angles + [pos_gripper]
            return pos
        elif self.motor_models[-1] == "robotiq":
            code, angles = self.api.get_servo_angle()
            code_gripper, pos_gripper = self.api.robotq_status['gFLT'], self.api.robotiq_status['gPO']
            pos = angles[:-1] + [pos_gripper]
            return pos

    def set_position_replay(self, position: np.ndarray):
        angles = position[:-1].tolist()
        gripper_pos = int(position[-1])

        if not self.replay_mode:
            self.api.set_mode(0)  # Set to follower mode
            self.api.set_state(0)  # Set to idle state
            self.replay_mode = True
        #angles = [round(angle,1) for angle in angles]  # Ensure angles are floats
        #angles = angles[:6]  # Ensure only the first 6 angles are used, as U850 has 6 joints + gripp

        
        # joints
        self.api.set_servo_angle(angle=angles, is_radian=False, wait=False, speed=self.MAX_SPEED_LIMIT, acceleration=self.MAX_ACC_LIMIT) 
        # gripper
        self.api.set_gripper_position(pos=gripper_pos, wait=False)

    def set_position(self,position: np.ndarray):
        angles = position[:-1].tolist()
        gripper_pos = int(position[-1])
        
        # joints        
        self.api.set_servo_angle_j(angles=angles, is_radian=False,wait = False )
                
        # gripper
        self.api.set_gripper_position(pos=gripper_pos, wait=False)


    def monitor_digital_input(self, stop_event):
        print("Starting monitor_digital_input")  # Debug print
        SINGLE_CLICK_TIME = 0.2
        DOUBLE_CLICK_TIME = 0.5
        LONG_CLICK_TIME = 1.0

        last_press_time = 0
        last_release_time = 0
        last_click_time = 0
        long_click_detected = False
        click_count = 0
        long_click_state = True  # starts in manual mode

        while not stop_event.is_set():
            try:
                if self.api is not None and self.is_connected:
                    code, value = self.api.get_tgpio_digital(ionum=2)
                    # print(f"Digital input read: code={code}, value={value}")  # Debug print
                    if code == 0:
                        current_time = time.time()
                        
                        if value == 1:  # Button pressed
                            if last_press_time == 0:
                                last_press_time = current_time
                            elif not long_click_detected and current_time - last_press_time >= LONG_CLICK_TIME:
                                print("Long click detected -> Switching manual mode")
                                long_click_detected = True
                                long_click_state = not long_click_state
                                if long_click_state:
                                    self.api.set_tgpio_digital(ionum=2, value=1)
                                    # manual mode
                                    self.api.clean_error()
                                    self.api.set_mode(2)
                                    self.api.set_state(0)
                                else:
                                    self.api.set_tgpio_digital(ionum=2, value=0)
                                    # disable manual mode
                                    self.api.clean_error()
                                    self.api.set_mode(0)
                                    self.api.set_state(0)
                        else:  # Button released
                            if last_press_time != 0:
                                press_duration = current_time - last_press_time

                                if not long_click_detected:
                                    if press_duration < SINGLE_CLICK_TIME:
                                        click_count += 1
                                        if click_count == 1:
                                            last_click_time = current_time
                                        elif click_count == 2:
                                            if current_time - last_click_time < DOUBLE_CLICK_TIME:
                                                if self.motor_models[-1] == "gipper":
                                                    print("Double click detected -> Open gripper")
                                                    self.api.set_gripper_position(pos=600, wait=False)  # Open gripper
                                                    click_count = 0
                                                elif self.motor_models[-1] == "robotiq":
                                                    print("Double click detected -> Open gripper")
                                                    self.api.robotiq_open()
                                                    click_count = 0
                                            else:
                                                if self.motor_models[-1] == "gipper":
                                                    print("Single click detected -> Close gripper")
                                                    self.api.set_gripper_position(pos=50, wait=False)  # Close gripper
                                                    click_count = 1
                                                elif self.motor_models[-1] == "robotiq":
                                                    print("Single click detected -> Close gripper")
                                                    self.api.robotiq_close()
                                                    click_count = 1
                                                last_click_time = current_time
                                    else:
                                        if click_count == 1 and self.motor_models[-1] == "gipper":
                                            print("Single click detected -> Close gripper")
                                            self.api.set_gripper_position(pos=50, wait=False)  # Close gripper
                                            click_count = 0
                                        elif click_count == 1 and self.motor_models[-1] == "robotiq":
                                            print("Single click detected -> Close gripper")
                                            self.api.robotiq_close()
                                            click_count = 0

                                last_release_time = current_time
                                last_press_time = 0
                                long_click_detected = False

                        # Reset click count if too much time has passed since last click
                        if click_count == 1 and current_time - last_click_time >= DOUBLE_CLICK_TIME and self.motor_models[-1] == "gipper":
                            print("Single click detected -> Close gripper")
                            self.api.set_gripper_position(pos=50, wait=False)  # Close gripper
                            click_count = 0
                        elif click_count == 1 and current_time - last_click_time >= DOUBLE_CLICK_TIME and self.motor_models[-1] == "robotiq":
                            print("Single click detected -> Close gripper")
                            self.api.robotiq_close()
                            click_count = 0

                else:
                    print("API not connected, waiting...")
                    time.sleep(1)  # Wait a bit longer before checking again
            except Exception as e:
                print(f"Error in monitor_digital_input: {e}")  # Debug print
            time.sleep(0.01)  # Check every 10ms for more precise detection
        print("Exiting monitor_digital_input")  # Debug print


    def robot_reset(self):
        if self.motor_models[-1] == "robotiq":
            self.api.robotiq_reset()
            raise ValueError(f"Unsupported gripper model: {self.motor_models[-1]}")
        elif self.motor_models[-1] == "gipper":
            self.api.set_gripper_position(pos=600, wait=True)