````md
# Unitree Go1 Navigation (ROS 2)

This repository contains a **ROS 2-based indoor mapping and navigation stack for the Unitree Go1 quadruped robot**.  
The project enables **SLAM-based map creation** and **autonomous navigation** using LiDAR, IMU, and odometry integration.

📌 **Repository**:  
https://github.com/Abhi-creator1/unitree_go1_nav

---

## 🚀 Features

- ✅ Real-time **SLAM mapping** using `slam_toolbox`
- ✅ Autonomous **navigation with Nav2**
- ✅ **LiDAR integration** (SLLIDAR)
- ✅ **Custom odometry & command bridge** for Unitree Go1
- ✅ Tested on **real Unitree Go1 hardware**
- ✅ ROS 2 Humble compatible

---

## 🧱 System Overview

**Hardware**
- Unitree Go1
- SLLIDAR (RPLIDAR compatible)
- Raspberry Pi (on robot)
- External control PC (Ubuntu + ROS 2)

**Software**
- ROS 2 Humble
- Nav2
- slam_toolbox
- Custom Unitree ROS 2 interfaces

---

## 📦 Required Packages

Make sure the following packages are installed:

```bash
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-tf-transformations
````

Clone required repositories into your ROS 2 workspace:

```bash
cd ~/unitree_ros2_ws/src
```

### Required Repositories

```bash
# SLAM
sudo apt install ros-humble-slam-toolbox

# LiDAR
git clone https://github.com/Slamtec/sllidar_ros2.git

# Unitree ROS2 (UPDATED VERSION REQUIRED)
git clone https://github.com/Abhi-creator1/unitree_ros2_to_real.git
```

⚠️ **Important**
Use **this updated version** of `unitree_ros2_to_real`, otherwise udp communication may not work correctly.


## 📂 Additional Files (IMPORTANT)

There is an **`additional/` folder** in this repository.

➡️ **All files inside `additional/` must be copied to the Raspberry Pi on the Go1 robot:**

```bash
~/ros_ws/src
```

These files are required for:

* Onboard Lidar data from unitree go1 to remote pc

---

## 🗺️ Mapping (SLAM)

Mapping is performed using **slam_toolbox** in online async mode.

### Launch Mapping

```bash
ros2 launch go1_nav mapping.launch.py
```

### What Happens

* LiDAR scans are published
* SLAM builds a 2D occupancy grid map
* Robot pose is estimated in real time
* Map is visualized in RViz

### Save the Map

After mapping the environment:

```bash
ros2 run nav2_map_server map_saver_cli -f unitree_map
```

This will generate:

* `unitree_map.pgm`
* `unitree_map.yaml`

Save them inside the `map/` directory.

---

## 🧭 Navigation

Navigation uses **Nav2** with a pre-built map.

### Launch Navigation

```bash
ros2 launch go1_nav navigation.launch.py
```

### Navigation Capabilities

* AMCL localization
* Global + local path planning
* Obstacle avoidance
* Goal-based navigation using RViz

### How to Navigate

1. Open RViz
2. Set **2D Pose Estimate**
3. Send **Nav2 Goal**
4. Go1 autonomously walks to the target

---

## 🔧 Custom Nodes Explained

| Node                            | Description                                        |
| ------------------------------- | -------------------------------------------------- |
| `cmd_vel_to_highcmd.cpp`        | Converts `/cmd_vel` to Unitree high-level commands |
| `unitree_highcmd_keepalive.cpp` | Keeps robot in active motion mode                  |
| `unitree_odom_node.cpp`         | Publishes odometry from Unitree sensors            |
| `imu_test.cpp`                  | IMU data validation and testing                    |

These nodes ensure **stable motion control** and **accurate localization**.

---

## ⚠️ Notes & Tips

* Ensure **TF tree** is correctly published (`map → odom → base_link`)
* Robot must be in **standing mode** before navigation
* Network latency can affect command smoothness
* Always test in **low-speed mode first**

---

## 📌 Tested Configuration

* **ROS 2**: Humble
* **OS**: Ubuntu 22.04
* **Robot**: Unitree Go1 (Real Hardware)
* **LiDAR**: SLLIDAR / RPLIDAR compatible

---

## 👤 Author

**Abhishek Thakur**
Robotics | Mechatronics | ROS 2
GitHub: [https://github.com/Abhi-creator1](https://github.com/Abhi-creator1)

```
```
