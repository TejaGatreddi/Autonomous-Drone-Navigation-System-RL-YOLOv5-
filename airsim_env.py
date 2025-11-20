#!/usr/bin/env python3
"""AirSim environment wrapper (Gym-style).
NOTE: This file is a connector/stub. To use: install AirSim and its Python client.
This implementation provides a Gym interface and fallback mode if AirSim not present.
"""
import os
import argparse
import numpy as np
import gym
from gym import spaces

try:
    import airsim
    _HAS_AIRSIM = True
except Exception:
    _HAS_AIRSIM = False

class AirSimDroneEnv(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(self, ip="127.0.0.1", image_shape=(84,84), discrete=False):
        super().__init__()
        self.image_shape = image_shape
        self.discrete = discrete
        # Observation: RGB image + simple telemetry (x,y,z,v)
        img_space = spaces.Box(low=0, high=255, shape=(*image_shape, 3), dtype=np.uint8)
        telemetry_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Dict({"image": img_space, "telemetry": telemetry_space})
        # Action: continuous 4-dim (pitch, roll, throttle, yaw-rate) in [-1,1]
        if discrete:
            self.action_space = spaces.Discrete(5)  # e.g. forward, left, right, up, hover
        else:
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        self.client = None
        if _HAS_AIRSIM:
            self.client = airsim.MultirotorClient(ip)
            self.client.confirmConnection()
        self.reset()

    def reset(self):
        # Reset simulation or return a default sample when AirSim missing
        if self.client:
            self.client.reset()
            self.client.enableApiControl(True)
            self.client.armDisarm(True)
            # takeoff
            self.client.takeoffAsync().join()
        self.state = {"pos": np.zeros(3), "vel": np.zeros(3)}
        # return initial observation
        obs = self._get_obs()
        return obs

    def step(self, action):
        # Map action to control; if no AirSim, use simple dynamics
        if self.client:
            # map continuous action to e.g. moveByVelocityAsync
            vx, vy, vz, yaw_rate = action
            duration = 0.5
            self.client.moveByVelocityAsync(vx*5, vy*5, vz*2, duration).join()
        else:
            # simple simulated dynamics
            self.state["vel"] = np.clip(action[:3], -3, 3)
            self.state["pos"] = self.state["pos"] + self.state["vel"] * 0.5
        obs = self._get_obs()
        reward, done, info = self._compute_reward()
        return obs, reward, done, info

    def _get_obs(self):
        # Acquire image if available
        if self.client:
            responses = self.client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
            img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
            if img1d.size == 0:
                image = np.zeros((*self.image_shape,3), dtype=np.uint8)
            else:
                image = img1d.reshape(responses[0].height, responses[0].width, 3)
                # resize to image_shape
                import cv2
                image = cv2.resize(image, (self.image_shape[1], self.image_shape[0]))
        else:
            # return a blank image for smoke runs
            image = np.zeros((*self.image_shape,3), dtype=np.uint8)
        telemetry = np.array([*self.state["pos"], np.linalg.norm(self.state["vel"])], dtype=np.float32)
        return {"image": image, "telemetry": telemetry}

    def _compute_reward(self):
        # Simplified reward: negative distance to goal minus collision penalty
        goal = np.array([10.0, 0.0, -2.0])
        pos = self.state["pos"]
        dist = np.linalg.norm(goal - pos)
        reward = -dist * 0.01
        done = False
        info = {}
        # If far below or collision (not simulated), end
        if dist < 1.0:
            reward += 10.0
            done = True
        # Clip steps
        return reward, done, info

    def render(self, mode="human"):
        pass

    def close(self):
        if self.client:
            self.client.armDisarm(False)
            self.client.enableApiControl(False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["smoke","live"], default="smoke")
    args = parser.parse_args()
    env = AirSimDroneEnv()
    obs = env.reset()
    print("Reset obs keys:", obs.keys())
