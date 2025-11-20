import numpy as np
from rl_envs.gym_drone_env import GymDroneEnv

def test_smoke_reset_and_step():
    env = GymDroneEnv(image_shape=(32,32), smoke=True)
    obs = env.reset()
    assert isinstance(obs, np.ndarray)
    action = env.action_space.sample()
    obs2, r, d, info = env.step(action)
    assert obs2.shape == obs.shape
