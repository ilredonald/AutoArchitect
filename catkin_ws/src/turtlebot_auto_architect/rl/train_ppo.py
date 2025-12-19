#!/usr/bin/env python3
import rospy
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
try:
    import gymnasium as gym  # pour Gymnasium moderne
except ImportError:
    import gym  # fallback vers Gym si Gymnasium non installé

from wall_follow_env import WallFollowEnv

# =========================
# Fonction de création d'environnement
# =========================
def make_env():
    return WallFollowEnv()

# =========================
# Entraînement PPO
# =========================
if __name__ == "__main__":
    rospy.init_node("train_ppo_node", anonymous=True)

    # Création de l'environnement vectorisé
    env = DummyVecEnv([make_env])

    # Instanciation du modèle PPO
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2
    )

    # Lancement de l'entraînement
    print("🏋️‍♂️ Début de l'entraînement PPO...")
    model.learn(total_timesteps=300_000)
    
    # Sauvegarde du modèle
    model.save("wall_tracer_ppo")
    print("✅ Entraînement terminé et modèle sauvegardé")

