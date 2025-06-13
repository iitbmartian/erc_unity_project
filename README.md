# ERC Unity Simulation

Simulating the Husarion Panther rover in ERC Mars Yard using Unity.

![Screenshot from 2025-06-13 17-59-38](https://github.com/user-attachments/assets/17faad7e-5d28-4f1d-a5ba-f609a040bca5)

## Prerequisites
- Ubuntu with ROS2 (any distro)
- A cpu with high core count and a dedicated nvidia gpu is recommended

## Getting Started
Download and Install [Unity Hub](https://unity.com/download). Once Unity Hub is installed it should ask you for login and licenses. Register for [Unity's Student Plan](https://unity.com/products/unity-student) and get your student license. It will then guide you through with activating that license in the Unity Hub.

Once Unity Hub is installed and license is activated you can git clone this repository and open Unity Hub and go to `Add` -> `Add project from disk`. Select erc_unity_project folder. It should then check the Unity editor version required to open that project and ask you to install it. Once the editor is installed, you can open the project and you are good to go.

Create a new ROS2 workspace and git clone [ROS-TCP-Endpoint package](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/main-ros2) in src. Build the package and run the following command.
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```
This will start the ROS TCP server on IP `127.0.0.1` and Port `10000`.

Now open the Unity Project and go to `Robotics` -> `ROS Settings`. Select ROS2 and enter the IP `127.0.0.1` and Port `10000`. Leave everything else the same and Start the simulation. If Everything is configured properly like mentioned and ROS-TCP-Endpoint is running on the same IP and Port, then a successful communication link between Unity and ROS2 will be setup.

>[!NOTE]
>Although the necessary Unity Packages should be installed by default, But if you dont see the `Robotics` tab or while running the simulation it shows a package missing, then make sure you download the necessary unity packages from the links provided at the end of this file.

Currently the Simulation is setup with an IMU and ZEDX. You should get imu data, rgb image, depth image and point cloud. Also odometry is setup which gives accurate odometry with zero deviation as it uses absolute position and rotation from unity to publish odometry data, you can turn odometry off from Inspector of `Panther` gameobject if you like to do odometry by yourself. Just click on any gameobject from the Hierarchy window and related scripts and other components should be visible in the unity's inspector window.

The robot moves with WASD and also subscribes to `/cmd_vel`, the code for movement is located in `Assets/Scripts/CarControl.cs`. You can modify the code according to your liking.

Once the simulation is running you can open RVIZ2 and view the data being published.

![Screenshot from 2025-06-13 18-02-22](https://github.com/user-attachments/assets/9129529f-b81d-4568-b10b-2a081ab09b8c)



## Unity Packages
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) is needed to communicate with ROS2.
- [UnitySensors Package](https://github.com/Field-Robotics-Japan/UnitySensors/tree/master) is a Sensor package provided by Field Robotics Japan, the IMU is taken from this package and other sensors are custom made. You may find this package usefull if you want to integrate any other sensors.
