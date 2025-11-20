#!/usr/bin/env python3
"""Entry-point to run training with a chosen env & algorithm.
Example:
  python src/run_training.py --env gym_drone_env --timesteps 200000
"""
import argparse
from rl_envs.gym_drone_env import GymDroneEnv
from agents.ppo_agent import make_agent, train, evaluate

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--env", default="rl_envs/gym_drone_env")
    parser.add_argument("--algo", default="ppo")
    parser.add_argument("--timesteps", type=int, default=100000)
    parser.add_argument("--out", default="models/ppo_drone.zip")
    args = parser.parse_args()

    # Use our Gym wrapper by default
    env = GymDroneEnv(image_shape=(64,64))
    agent = make_agent(policy="MlpPolicy", env=env)
    train(agent, total_timesteps=args.timesteps, save_path=args.out, checkpoint_freq=max(10000, args.timesteps//5))
    metrics = evaluate(agent, env, n_episodes=5)
    print("Evaluation metrics:", metrics)

if __name__ == "__main__":
    main()
