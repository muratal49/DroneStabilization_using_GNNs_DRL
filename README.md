# 🛰️ Drone Stabilization using Reinforcement Learning

This repository contains the simulation and training code for an autonomous drone that can **self-stabilize mid-air** after being dropped from an aircraft.  
The project was developed as part of an internship project focused on **reinforcement learning (RL)**, **simulation environments**, and **aerodynamic modeling** using PyBullet and Stable-Baselines3.

---

## 🚀 Project Overview

The goal is to train a quadrotor drone to:
1. **Wake up after being dropped** (free fall initialization)
2. **Stabilize itself against tumbling**
3. **Recover upright flight**
4. **Fly and land autonomously** under varying wind, payload, and turbulence conditions.

The project uses a **Curriculum Learning** approach to progressively increase difficulty (wind, payload, angular velocity) and trains the RL agent using **PPO** from Stable-Baselines3.

---

## 🧩 Repository Structure

```
📦 DroneStabilization_using_GNNs_DRL
├── july31_train.py                 # Main training script (PPO Curriculum RL)
├── july31_dashboard.py             # Dashboard for monitoring and visualization
├── july31_CurriculumHoverAviary.py # Custom Gym-PyBullet environment
├── requirements.txt                # Dependencies
├── README.md                       # Project documentation
└── data/                           # (optional) logs, model checkpoints, plots
```

---

## ⚙️ Setup Instructions

### 1️⃣ Clone the Repository
```bash
git clone https://github.com/muratal49/DroneStabilization_using_GNNs_DRL.git
cd DroneStabilization_using_GNNs_DRL
```

### 2️⃣ Create and Activate a Conda Environment
```bash
conda create -n drone_rl python=3.10 -y
conda activate drone_rl
```

### 3️⃣ Install Requirements
```bash
pip install -r requirements.txt
```

---

## 🧠 Training the Model

Run the PPO-based curriculum training:
```bash
python july31_train.py
```

You can monitor the progress in TensorBoard:
```bash
tensorboard --logdir ./logs
```

---

## 📊 Visual Dashboard

After training, visualize stabilization metrics (altitude, angular velocities, reward trends):

```bash
python july31_dashboard.py
```

You can also generate flight trajectories and 3D plots of the drone’s orientation recovery.

---

## 🪂 Environment Design

The custom Gym environment `CurriculumHoverAviary` is built on top of **Gym-PyBullet-Drones** and defines:
- 3D dynamics with free fall, tumbling, and wind disturbance
- PID-based low-level control
- Reward shaping for stability and energy efficiency
- Multi-phase curriculum progression

---

## 📈 Example Visualization

Example TensorBoard curves (reward vs. steps):

```
tensorboard --logdir=logs
```

You can expect:
- Rapid reward convergence after 1.5M steps
- Stable attitude control under ±2π rad/s disturbances
- Smooth altitude recovery under 40 m/s wind

---

## 📦 requirements.txt

Below are the dependencies required for training and visualization:

```text
torch>=2.0.0
numpy>=1.24.0
matplotlib>=3.7.0
pandas>=2.0.0
gymnasium>=0.28.1
pybullet>=3.2.5
stable-baselines3>=2.2.1
tqdm>=4.66.0
tensorboard>=2.14.0
opencv-python>=4.8.0
scipy>=1.11.0
seaborn>=0.12.2
```

---

## 📚 References
- [Gym-PyBullet-Drones](https://github.com/utiasDSL/gym-pybullet-drones)
- [Stable-Baselines3 Documentation](https://stable-baselines3.readthedocs.io/)
- [CMU KiltHub Drone Flight Dataset](https://kilthub.cmu.edu/articles/dataset/)
- [Zenodo Flight Logs Dataset](https://zenodo.org/records/13682870)

---

## 👤 Author

**Murat Al**  
M.Sc. Data Science, University of Rochester  
Ph.D. Computational Mechanics, Lehigh University  
🔗 [LinkedIn](https://www.linkedin.com/in/muratal49) | [GitHub](https://github.com/muratal49)

---

> 🧩 *“A drone that learns to recover itself is the first step toward truly autonomous aerial systems.”*
