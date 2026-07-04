# 🎯 Autonomous Shooter Robot (ROS 2 & Gazebo)

An advanced autonomous robotics simulation built with ROS 2 (Humble) and Gazebo (Ignition). This project features a custom mobile robot that uses `Nav2` to autonomously navigate a warehouse environment, lock onto targets using camera and image processing (cv2) , and fire physical projectiles calculated via 3D kinematics.

## 🎥 Demonstration Video

https://github.com/user-attachments/assets/2740177c-4218-49c3-865b-169265c9b60e

---

## 🚀 Key Features

* **Autonomous Navigation (Nav2):** Uses the `BasicNavigator` API to dispatch the robot to sequential waypoints (Pickup, Dropoff, Dock) across a mapped warehouse environment.
* **Custom Action Server (`ShootTarget`):** A multithreaded ROS 2 Action Server that handles target locking, LiDAR depth extraction, and execution without blocking the main navigation thread.
* **Sensor Fusion & Targeting:** Maps the turret's pan/tilt angles to a 360-degree LiDAR scan (`/scan`), utilizing a multi-ray cone to accurately extract the target's distance.

---

## 📂 Repository Structure

```text
autonomous_shooter/
├── config/
│   └── mapper_params_online_async.yaml   # SLAM configuration
├── maps/
│   ├── nav2_config.yaml                  # Nav2 parameter overrides
│   ├── warehouse_layout.pgm              # 2D Occupancy grid map
│   └── warehouse_layout.yaml             # Map metadata
├── rviz/
│   └── rviz_config.rviz                  # RViz2 visualization setup
├── src/
│   ├── custom_interfaces/                # Custom ROS 2 Messages/Actions
│   │   ├── action/
│   │   │   └── ShootTarget.action        # Action definition for targeting
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── robot_setup/                      # Main Python Logic Package
│       ├── launch/
│       │   └── system_bringup.launch.py  # Master launch file
│       ├── robot_setup/
│       │   ├── __init__.py
│       │   ├── detect_shoot.py           # Action Server: Target locking & firing
│       │   └── mission.py                # Action Client & Nav2 Waypoint logic
│       ├── urdf/                         # Robot description files (XACRO/URDF)
│       ├── setup.py
│       └── package.xml
└── world/
    ├── warehouse.sdf                     # Gazebo simulation environment
    └── warehouse.sdf.xacro
```

---

## 🛠️ Prerequisites

Ensure your system (or container) has the following dependencies installed:

* **OS:** Ubuntu 22.04 (Jammy Jellyfish)
* **ROS 2:** Humble Hawksbill
* **Simulator:** Ignition Gazebo (Fortress)
* **System Packages:** `nav2_simple_commander`, `ros_gz_bridge`, `ros_gz_sim`
* **Python Packages:** `opencv-python` (cv2)

---

## ⚙️ Build Instructions

Because this workspace contains custom action interfaces (`ShootTarget.action`), it must be built cleanly from the root directory.

1. **Navigate to the workspace:**
   ```bash
   cd ~/autonomous_shooter
   ```

2. **Resolve any missing dependencies:**
   ```bash
   rosdep update
   rosdep install --from-paths src -y --ignore-src
   ```

3. **Build the workspace:**
   *(Note: Using `--symlink-install` allows you to edit Python scripts without having to rebuild the workspace every single time).*
   ```bash
   colcon build --symlink-install
   ```
   > **Note on CMake Cache:** If you ever rename the root workspace folder, CMake cache paths will break. Run `rm -rf build/ install/ log/` to clear the old paths before running the build command again. Also make sure to change it in the system_bringup.launch.py file.

4. **Source the new overlay:**
   ```bash
   source install/setup.bash
   ```

---

## 🎮 Usage

Running the full autonomous pipeline requires three separate terminals. **Make sure you run `source install/setup.bash` in every new terminal before executing these commands!**

**Terminal 1: Launch the Simulation Environment**
This master launch file brings up Ignition Gazebo, RViz, Nav2, the Robot State Publisher, and all necessary ROS-Gazebo message bridges.
```bash
ros2 launch robot_setup system_bringup.launch.py
```

**Terminal 2: AMCL Convergence**
Drive the robot manually a few metres to converge the AMCL particle cloud.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Terminal 3: Execute the Autonomous Mission**
Launch the main mission script. The Nav2 commander will sequentially drive the robot to the Target Locations. Upon arriving at each station, it will call the Action Server to scan, lock, and fire at the target.
```bash
ros2 run robot_setup warehouse_mission
```

