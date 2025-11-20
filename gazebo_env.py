#!/usr/bin/env python3
"""Gazebo environment adapter (stub).
This file is intentionally short: it provides an interface and guidance.
To use with ROS/Gazebo, launch your Gazebo world and adjust topic names in the ROS nodes.
"""
import gym
from gym import spaces
import numpy as np

class GazeboDroneEnv(gym.Env):
    def __init__(self, image_shape=(84,84)):
        super().__init__()
        img_space = spaces.Box(low=0, high=255, shape=(*image_shape,3), dtype=np.uint8)
        telemetry_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Dict({"image": img_space, "telemetry": telemetry_space})
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        self.state = {"pos": np.zeros(3), "vel": np.zeros(3)}

    def reset(self):
        self.state = {"pos": np.zeros(3), "vel": np.zeros(3)}
        return self._get_obs()

    def step(self, action):
        self.state["pos"] += action[:3] * 0.5
        obs = self._get_obs()
        reward = -np.linalg.norm(self.state["pos"] - np.array([10,0,-2]))*0.01
        done = False
        return obs, reward, done, {}

    def _get_obs(self):
        img = (np.random.rand(*self.observation_space["image"].shape) * 255).astype('uint8')
        tele = np.array([*self.state["pos"], np.linalg.norm(self.state["vel"])], dtype=np.float32)
        return {"image": img, "telemetry": tele}
