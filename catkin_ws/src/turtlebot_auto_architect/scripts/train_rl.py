#!/usr/bin/env python3
from stable_baselines3 import PPO
from auto_architect_env import AutoArchitectEnv

env = AutoArchitectEnv()

model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    learning_rate=3e-4,
    gamma=0.99
)

model.learn(total_timesteps=100_000)
model.save("auto_architect_rl")
