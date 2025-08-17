# ERC Unity Simulation

Simulating the **Husarion Panther rover** in the **ERC Mars Yard** using **Unity** and **ROS2**.

![Simulation Screenshot](https://github.com/user-attachments/assets/4da08646-c4d3-4315-9422-7cc5cc0e8ae2)




## Prerequisites

- **Operating System:** Ubuntu (with ROS2 installed, humble/jazzy)
- **Hardware:** A high-core-count CPU and a dedicated NVIDIA GPU is recommended



## Getting Started

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

Build the ros2 ws:
```bash
cd erc_ws
colcon build
source install/setup.bash
```

Launch the TCP server:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```

Launch the static TF publisher:

```bash
ros2 run unity_sim static_tf_publisher
```

### 4. Connect Unity to ROS2

In Unity:
- Go to `Robotics` → `ROS Settings`
- Select `ROS2`
- Set `ROS IP` to `127.0.0.1`
- Set `ROS TCP Port` to `10000`
- Start the simulation

> [!TIP]
> If the `Robotics` tab is missing or packages are not found, refer to the [Unity Packages](#unity-packages) section to download the required packages.



## Sensor Integration

This simulation includes multiple sensors with data published via ROS2 topics.

### IMU Sensor
- **Source:** UnitySensors package
- **Data Published:**
  - `/panther/imu/data` - IMU data

### LS LIDAR C16
- **Data Published:**
  - `/panther/cx/lslidar_point_cloud` - Lidar point cloud
 
### Cameras
- **Data Published:**
  - `/panther/camera_front/image_raw` - Front camera
  - `/panther/camera_back/image_raw` - Back camera
  - `/panther/camera_left/image_raw` - Left camera
  - `/panther/camera_right/image_raw` - Right camera

### Odometry
- **Data Published:** `/panther/odometry/filtered`
- **Behavior:** Uses Unity’s absolute position and rotation to publish accurate odometry with **zero drift**
- **Toggle:** Can be disabled from the `Panther` GameObject’s Inspector



## Controls

- Use **WASD** keys to drive the rover using Unity
- Also subscribes to `/panther/cmd_vel`, so you can use Teleop keyboard or any other interface that publishes TwistStamped msg to `/panther/cmd_vel`
- **PID Controller:** The rover movement is now controlled using a PID system, making driving via `/panther/cmd_vel` smooth and realistic

Movement logic is located in:
```
Assets/Scripts/CarControl.cs
```

You can modify this script to customize the robot's control logic.



## Visualizing in RViz2

Once the simulation is running and data is publishing, you should be able to visualize the following in RVIZ2:
- `/panther/odometry/filtered`
- `/panther/cx/lslidar_point_cloud`
- `/panther/camera_front/image_raw`
- `/panther/camera_back/image_raw`
- `/panther/camera_left/image_raw`
- `/panther/camera_right/image_raw`

![RViz2 Screenshot](https://github.com/user-attachments/assets/f0a4b044-98ed-4ed6-8f9b-2d0b10701a9e)




## Environment & Objects

- The world contains **ArUco markers** placed throughout the Mars Yard for testing localization and perception
- Additional interactive objects have been added:
  - **Mallet**
  - **Cone**
  - **Screwdriver**
- These are useful for developing and benchmarking the object detection and identification pipeline


## Performance Notes

- The simulation is highly **optimized** to run at a stable frame rate
- All ROS2 topics are published **without frame drops**
- The topic types and names are **exactly the same as the real Husarion Panther rover**, ensuring seamless integration with existing pipelines


## Unity Packages

- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) – for ROS2 communication
- [UnitySensors](https://github.com/Field-Robotics-Japan/UnitySensors/tree/master) – for IMU and other cool sensors that you can directly integrate if you want


## Troubleshooting

- **No data in RViz2:** Check if ROS TCP Endpoint is running on correct IP/port, and correct topic is selected. For more help refer to [ROS Unity Integration](https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main/tutorials/ros_unity_integration).
- **Simulation lagging:** Try reducing the frame rate and publishing frequency to ros2
