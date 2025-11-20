"""PPO agent wrapper using Stable-Baselines3.
This keeps training & evaluation code minimal and reproducible.
"""
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
import os

def make_agent(policy="MlpPolicy", env=None, lr=3e-4):
    model = PPO(policy, env, verbose=1, learning_rate=lr, tensorboard_log="./tb")
    return model

def train(agent, total_timesteps=100000, save_path="models/ppo_drone.zip", checkpoint_freq=50000):
    os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
    checkpoint_cb = CheckpointCallback(save_freq=checkpoint_freq, save_path=os.path.dirname(save_path), name_prefix="ppo_checkpoint")
    agent.learn(total_timesteps=total_timesteps, callback=checkpoint_cb)
    agent.save(save_path)
    print("Saved agent to", save_path)

def evaluate(agent, env, n_episodes=10):
    rewards = []
    for _ in range(n_episodes):
        obs = env.reset()
        done = False
        ep_r = 0.0
        while not done:
            action, _ = agent.predict(obs, deterministic=True)
            obs, r, done, info = env.step(action)
            ep_r += r
        rewards.append(ep_r)
    return {"mean_reward": sum(rewards)/len(rewards), "rewards": rewards}
