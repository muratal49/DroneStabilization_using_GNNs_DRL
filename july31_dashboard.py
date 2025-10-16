import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import imageio
import time
import pybullet as p
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from gym_pybullet_drones.utils.enums import ActionType, ObservationType
from july31_CurriculumHoverAviary import CurriculumHoverAviary_july31

st.set_page_config(layout="wide")
st.title("🚀 July 31 Curriculum-Trained Drone Dashboard")

@st.cache_resource
def load_model():
    env = CurriculumHoverAviary_july31(gui=False, act=ActionType.RPM, obs=ObservationType.KIN)
    env = Monitor(env)
    model = PPO.load("ppo_july31_final.zip", env=env)
    env.close()
    return model

model = load_model()

cols = st.columns(6)
wind_speed = cols[0].slider("Wind Speed (m/s)", 0.0, 40.0, 10.0, 0.5)
payload = cols[1].slider("Payload (g)", 0, 1500, 500, 10)
tumble = cols[2].slider("Tumbling (rad/s)", 0.0, 8.0, 2.0, 0.5)
ang_x = cols[3].number_input("Angular Vel X", value=2.0)
ang_y = cols[4].number_input("Angular Vel Y", value=2.0)
ang_z = cols[5].number_input("Angular Vel Z", value=1.0)

if st.button("▶️ Run Simulation"):
    env = CurriculumHoverAviary_july31(
        wind_speed_range=(wind_speed, wind_speed),
        payload_range=(payload, payload),
        tumbling_range=(-tumble, tumble),
        gui=False,
        act=ActionType.RPM,
        obs=ObservationType.KIN
    )
    env = Monitor(env)
    obs, _ = env.reset()

    drone_id = env.unwrapped.DRONE_IDS[0]
    client = env.unwrapped.CLIENT
    p.resetBasePositionAndOrientation(drone_id, [0, 0, 300], [0, 0, 0, 1], physicsClientId=client)
    p.resetBaseVelocity(drone_id, angularVelocity=[ang_x, ang_y, ang_z], physicsClientId=client)
    p.stepSimulation(physicsClientId=client)
    env.unwrapped._updateAndStoreKinematicInformation()

    done, frames, heights, timesteps = False, [], [], []
    total_reward, step = 0, 0

    while not done and step < 2000:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, _ = env.step(action)
        done = terminated or truncated
        total_reward += reward
        step += 1

        state = env.unwrapped._getDroneStateVector(0)
        heights.append(state[2])
        timesteps.append(step * env.unwrapped.CTRL_TIMESTEP)

        img = p.getCameraImage(640, 480,
                               viewMatrix=p.computeViewMatrixFromYawPitchRoll(state[0:3], 1.5, 45, -20, 0, 2),
                               projectionMatrix=p.computeProjectionMatrixFOV(60, 640/480, 0.1, 500),
                               renderer=p.ER_TINY_RENDERER)
        rgba = np.array(img[2], dtype=np.uint8).reshape(img[1], img[0], 4)
        frames.append(rgba[:, :, :3])
        time.sleep(0.01)

    env.close()

    gif_path = "july31_simulation.gif"
    imageio.mimsave(gif_path, frames, duration=0.2)

    st.image(gif_path)
    fig, ax = plt.subplots()
    ax.plot(timesteps, heights, label="Height")
    ax.axhline(30, color="r", linestyle="--", label="30m Target")
    ax.legend(); ax.grid()
    st.pyplot(fig)
