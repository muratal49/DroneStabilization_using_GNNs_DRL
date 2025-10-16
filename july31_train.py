import os
import gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.monitor import Monitor
from gym_pybullet_drones.utils.enums import ActionType, ObservationType
from july31_CurriculumHoverAviary import CurriculumHoverAviary_july31  # ✅ your custom env

def make_env(wind, payload, tumble):
    def _init():
        env = CurriculumHoverAviary_july31(
            wind_speed_range=(0, wind),
            payload_range=(0, payload),
            tumbling_range=(-tumble, tumble),
            act=ActionType.PID,
            obs=ObservationType.KIN,
            gui=False,
            record=False
        )
        env = Monitor(env)
        return env
    return _init

if __name__ == "__main__":   # ✅ CRITICAL FIX
    curriculum = [
        {"wind": 5,  "payload": 300,  "tumble": 2},
        {"wind": 15, "payload": 700,  "tumble": 4},
        {"wind": 25, "payload": 1000, "tumble": 6},
        {"wind": 40, "payload": 1500, "tumble": 8}
    ]

    model = None

    for i, params in enumerate(curriculum):
        print(f"🚀 Starting curriculum phase {i+1}: {params}")
        
        env = SubprocVecEnv([
            make_env(params["wind"], params["payload"], params["tumble"])
            for _ in range(4)  # ✅ 4 parallel envs
        ])

        if model is None:
            model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_tb/")
        else:
            model.set_env(env)

        model.learn(total_timesteps=500_000)  # adjust as needed
        model.save(f"ppo_july31_phase{i+1}.zip")  # ✅ Save after each phase

        env.close()

    print("✅ Final model trained and saved as ppo_july31_final.zip")
    model.save("ppo_july31_final.zip")
