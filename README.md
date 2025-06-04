# Drone Flight Sensor Analysis with GNNs

This repository explores the application of Graph Neural Networks (GNNs) to multi-modal drone flight data for stability estimation, state representation, and control modeling. The goal is to use real-world sensor logs to learn structured representations of drone states, and ultimately develop data-driven models for robust flight analysis and control under dynamic outdoor conditions.

## 📊 Dataset

The dataset used in this project is publicly available on Zenodo:

Grigoriou, Y., Souli, N., Kardaras, P., & Kolios, P. (2024).  
Drone onboard multi-modal sensor dataset for complex outdoor scenarios (Version 1) [Data set].  
Zenodo. https://doi.org/10.5281/zenodo.13682870

The dataset includes 20 outdoor drone flights, each with time-series logs of multiple onboard sensors such as:

- Inertial Measurement Unit (IMU): linear acceleration, angular velocity
- Environmental: wind speed and angle
- Positioning: position_x/y/z
- System: battery voltage/current, payload, yaw
- Flight logic states: IDLE_HOVER, ASCEND, TURN, etc.

Each flight is sampled sequentially and annotated with flight phase labels.

## 🎯 Objectives

- Build a graph-based representation of sensor states for each timestep
- Use GNNs to model inter-sensor dependencies
- Predict control-relevant outputs such as yaw or flight state
- Explore hybrid models with Reinforcement Learning (planned)

## 🧠 Methods (Planned)

We explore:
- Static Graph Neural Networks (GCN, GAT) over sensor features
- Sequential modeling using GNNs across time steps
- Flight mode classification and yaw prediction tasks

All models are implemented using PyTorch and PyTorch Geometric.

## 📁 Project Structure


├──  data/ # Raw and preprocessed CSV files 
├──  notebooks/ # Jupyter notebooks for EDA, preprocessing, and modeling 
├──  models/ # GNN models and training scripts 
├── utils/ # Helper functions for graph construction and preprocessing 
├── README.md 
└── requirements.txt 


