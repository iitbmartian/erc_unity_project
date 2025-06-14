# ERC Unity Simulation

Simulating the **Husarion Panther rover** in the **ERC Mars Yard** using **Unity** and **ROS2**.

![Simulation Screenshot](https://github.com/user-attachments/assets/17faad7e-5d28-4f1d-a5ba-f609a040bca5)



## 📋 Prerequisites

- **Operating System:** Ubuntu (with ROS2 installed, any distro)
- **Hardware:** A high-core-count CPU and a dedicated NVIDIA GPU is recommended



## 🚀 Getting Started

### 1. Clone This Repository

```bash
git clone https://github.com/iitbmartian/erc_unity_project.git
cd erc_unity_project
```
### 2. Install Unity Hub and Unity Editor

Download and Install [Unity Hub](https://unity.com/download). Once Unity Hub is installed it should ask you for login and licenses. Register for [Unity's student plan](https://unity.com/products/unity-student) and get your free student license. It will then guide you through with activating that license in the Unity Hub.

Once installed:
- Open Unity Hub → `Add` → `Add project from disk`
- Select the cloned `erc_unity_project` folder
- Unity will prompt you to install the required editor version


### 3. Set Up ROS2 TCP Communication

Create a ROS2 workspace (if you don't have one):

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone the [ROS-TCP-Endpoint package](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/main-ros2):

```bash
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ..
colcon build
source install/setup.bash
```

Launch the TCP server:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```

### 4. Connect Unity to ROS2

In Unity:
- Go to `Robotics` → `ROS Settings`
- Select `ROS2`
- Set `ROS IP` to `127.0.0.1`
- Set `ROS TCP Port` to `10000`
- Start the simulation

> [!TIP]
> If the `Robotics` tab is missing or packages are not found, refer to the [Unity Packages](#-unity-packages) section to download the required packages.



## 🧭 Sensor Integration

This simulation includes multiple sensors with data published via ROS2 topics.

### 🧠 IMU Sensor
- **Source:** UnitySensors package
- **Data Published:**
  - `/imu/data`

### 📷 ZED-X RGB Camera
- **Data Published:**
  - `/zedx/left/image_raw` – RGB image feed
  - `/zedx/left/depth_raw` – depth image
  - `/zedx/points` – point cloud

### 📍 Odometry
- **Data Published:** `/odom`
- **Behavior:** Uses Unity’s absolute position and rotation to publish accurate odometry with **zero drift**
- **Toggle:** Can be disabled from the `Panther` GameObject’s Inspector



## 🎮 Controls

- Use **WASD** keys to drive the rover using Unity
- Also subscribes to `/cmd_vel`, so you can use Teleop keyboard or any other interface that publishes to `/cmd_vel`

Movement logic is located in:
```
Assets/Scripts/CarControl.cs
```

You can modify this script to customize the robot's control logic.



## 📡 Visualizing in RViz2

Once the simulation is running and data is publishing, you should be able to visualize the following in RVIZ2:
- `/odom`
- `/imu/data`
- `/zedx/left/image_raw`
- `/zedx/left/depth_raw`
- `/zedx/points`

![RViz2 Screenshot](https://github.com/user-attachments/assets/9129529f-b81d-4568-b10b-2a081ab09b8c)



## 📦 Unity Packages

- 🔌 [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) – for ROS2 communication
- 🎯 [UnitySensors](https://github.com/Field-Robotics-Japan/UnitySensors/tree/master) – for IMU and other cool sensors that you can directly integrate if you want


## 🛠 Troubleshooting

- **No data in RViz2:** Check if ROS TCP Endpoint is running on correct IP/port, and correct topic is selected. For more help refer to [ROS Unity Integration](https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main/tutorials/ros_unity_integration).
- **Simulation lagging:** Try reducing the frame rate and publishing frequency to ros2
