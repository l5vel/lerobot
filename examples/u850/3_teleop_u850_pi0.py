import time
import cv2
import tqdm
from lerobot.common.robot_devices.motors.ufactory import xArmWrapper
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.cameras.opencv import OpenCVCamera
from lerobot.common.robot_devices.cameras.intelrealsense import IntelRealSenseCamera


from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.datasets.populate_dataset import (
    create_lerobot_dataset,
    delete_current_episode,
    init_dataset,
    save_current_episode,
)

# Defines how to communicate with the motors of the leader and follower arms
leader_arms = {
    "main": xArmWrapper(
        port="172.16.0.13",
        motors={
            # name: (index, model)
            "joint1": (1, "ufactory-850"),
            "joint2": (2, "ufactory-850"),
            "joint3": (3, "ufactory-850"),
            "joint4": (4, "ufactory-850"),
            "joint5": (5, "ufactory-850"),
            "joint6": (6, "ufactory-850"),
            "gripper": (7, "ufactory-850"),
        },
    ),
}
follower_arms = {
    "main": xArmWrapper(
        port="172.16.0.11",
        motors={
            # name: (index, model)
            "joint1": (1, "ufactory-850"),
            "joint2": (2, "ufactory-850"),
            "joint3": (3, "ufactory-850"),
            "joint4": (4, "ufactory-850"),
            "joint5": (5, "ufactory-850"),
            "joint6": (6, "ufactory-850"),
            "gripper": (7, "ufactory-850"),
        },
    ),
}
robot = ManipulatorRobot(
    robot_type="u850",
    calibration_dir=".cache/calibration/u850",
    cameras={
        "top": OpenCVCamera(8, fps=30, width=640, height=480),
        "wrist": OpenCVCamera(6,fps = 30, width = 640, height = 480),#, use_depth = True), ## apparently Opencvcamera 6 is realsense  camera
    },    
    leader_arms=leader_arms,
    follower_arms=follower_arms,
)

# Connect motors buses and cameras if any (Required)
robot.connect()
print('lol')

try:
    while True:
        # robot.teleop_step()
        time.sleep(0.004)  # 250 Hz

        # # Recording data, only joints
        #leader_pos = robot.leader_arms["main"].get_position()
        #follower_pos = robot.follower_arms["main"].get_position()
        observation, action = robot.teleop_step(record_data=True)
        #print(observation)
        # print(follower_pos)
        # print(observation)
        # print(leader_pos)
        # print(action)
        # print("---")

        # Recording data with cameras
        # observation, action = robot.teleop_step(record_data=True)
        # print(observation["observation.images.top"].shape)
        # print(observation["observation.images.wrist"].shape)
        # image_keys = [key for key in observation if "image" in key]
        # for key in image_keys:
        #     print("trying to cv show")
        #     cv2.imshow(key, cv2.cvtColor(observation[key].numpy(), cv2.COLOR_RGB2BGR))
        #     cv2.waitKey(1)
        # print(observation["observation.images.wrist"][0][0][2])
        # print(observation["observation.images.wrist"].min().item())
        # print(observation["observation.images.wrist"].max().item())
        # print("---")
        #time.sleep(0.033)  # 30 Hz -> barely smooth

except KeyboardInterrupt:
    print('Operation interrupted by user.')

# seconds = 30
# frequency = 200
# for _ in tqdm.tqdm(range(seconds*frequency)):
#     robot.teleop_step()

robot.disconnect()
