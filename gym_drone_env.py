#!/usr/bin/env python3
"""Gym wrapper that formats the observation for RL libraries (concatenate image & telemetry into a flat vector).
Also provides a smoke mode for quick tests.
"""
import argparse
import numpy as np
import gym
from gym import spaces
from sim.gazebo_env import GazeboDroneEnv

class GymDroneEnv(gym.Env):
    def __init__(self, image_shape=(84,84), smoke=False):
        super().__init__()
        self.inner = GazeboDroneEnv(image_shape=image_shape)
        img_shape = image_shape + (3,)
        # For simplicity we flatten the image into a vector (not ideal but quick)
        obs_len = img_shape[0]*img_shape[1]*img_shape[2] + 4
        self.observation_space = spaces.Box(low=0, high=255, shape=(obs_len,), dtype=np.float32)
        self.action_space = self.inner.action_space
        self.smoke = smoke

    def reset(self):
        o = self.inner.reset()
        return self._format_obs(o)

    def step(self, action):
        o, r, d, info = self.inner.step(action)
        return self._format_obs(o), r, d, info

    def _format_obs(self, o):
        img = o["image"].astype(np.float32).flatten() / 255.0
        tele = o["telemetry"].astype(np.float32)
        obs = np.concatenate([img, tele.astype(np.float32)], axis=0)
        return obs

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["smoke","run"], default="smoke")
    args = parser.parse_args()
    env = GymDroneEnv(smoke=(args.mode=="smoke"))
    obs = env.reset()
    print("Obs shape:", obs.shape)
