# 🛸 Autonomous Drone Navigation System (RL + YOLOv5)

An intelligent reinforcement learning–powered drone navigation project that enables autonomous flight, obstacle avoidance, and real-time perception using YOLOv5.  
This system integrates simulation (AirSim/Gazebo), ROS, and deep reinforcement learning to train drones to navigate safely in complex environments.

---

## 🚀 Overview

**Autonomous Drone Navigation System** is designed to train UAVs to understand their environment, avoid obstacles, and reach destination points using advanced machine learning.  
The project combines **Reinforcement Learning (PPO)**, **Object Detection (YOLOv5)**, and **Simulation Environments** to create a scalable and safe drone navigation pipeline.

---

## 🧩 Features

- 🤖 **Reinforcement Learning (PPO)** — Drone learns optimal navigation behavior through training.  
- 🛰️ **AirSim/Gazebo Simulation** — Realistic physics environments for safe training.  
- 👁️ **YOLOv5 Object Detection** — Real-time recognition of obstacles and dynamic objects.  
- ⚙️ **ROS Integration** — Perception and control nodes for real-time deployment.  
- 📡 **Modular Architecture** — Easily extendable for real-world drones and custom hardware.  
- 🧪 **Smoke Testing Mode** — Quick tests even without simulators installed.

---

## 🧱 Project Structure

```
autonav-drone/
│
├── sim/                     # AirSim/Gazebo environment connectors
│   ├── airsim_env.py       
│   └── gazebo_env.py
│
├── src/
│   ├── rl_envs/             # Gym-compatible RL environment
│   │   └── gym_drone_env.py
│   ├── agents/              # PPO RL agent
│   │   └── ppo_agent.py
│   ├── detection/           # YOLOv5 wrapper
│   │   └── yolo_wrapper.py
│   ├── ros_nodes/           # ROS-based perception & control nodes
│   │   ├── perception_node.py
│   │   └── control_node.py
│   ├── utils/               # Metrics, replay recorder
│   └── run_training.py      # Training entry point
│
├── docs/
├── tests/
├── requirements.txt
├── Dockerfile
├── README.md
└── LICENSE
```

---

## ⚙️ Installation

### 1️⃣ Clone this repository
```bash
git clone https://github.com/yourusername/autonav-drone.git
cd autonav-drone
```

### 2️⃣ Create & activate virtual environment
```bash
python -m venv venv
source venv/bin/activate       # Mac/Linux
venv\Scripts\activate          # Windows
```

### 3️⃣ Install dependencies
```bash
pip install -r requirements.txt
```

---

## ▶️ Usage

### 🛠️ Run the RL training
```bash
python src/run_training.py --timesteps 200000
```

### 👁️ Run YOLOv5 perception node (ROS)
```bash
python src/ros_nodes/perception_node.py
```

### 🛫 Run control node (ROS)
```bash
python src/ros_nodes/control_node.py
```

### 🧪 Run smoke-test environment (no simulator needed)
```bash
python sim/airsim_env.py --mode smoke
```

---

## 📈 Example Output

| Component      | Output |
|----------------|--------|
| RL Reward      | Increasing trend as drone learns optimal paths |
| YOLOv5 Output  | Labels + bounding boxes for obstacles |
| Drone Control  | Real-time `/cmd_vel` commands |

---

## 🧮 Tech Stack

- **ML & RL:** PyTorch, Stable-Baselines3  
- **Simulation:** AirSim / Gazebo  
- **Detection:** YOLOv5  
- **Middleware:** ROS (rospy)  
- **Language:** Python  
- **Visualization:** Matplotlib  

---

## 🧑‍💻 Author

**TEJENDRA GATREDDI**  
📍 AI | ML | Robotics | Reinforcement Learning  
📧 tgatredd@gitam.in  

---

## 🪪 License

This project is licensed under the **MIT License**.

---

## ⭐ Contributing

Contributions are welcome!  
Open an issue or submit a pull request to enhance the system.

---

## 🌟 Acknowledgements

- Microsoft AirSim  
- Gazebo Simulation  
- Ultralytics YOLO  
- Reinforcement Learning Community  

---

> “Autonomy is achieved when perception, learning, and control work as one.”
