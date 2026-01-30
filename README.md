# 🤖 Erobotics Workspace HOla

MyCobot 280 Control with ROS 2

This repository contains a ROS 2 Jazzy workspace configured to control the **MyCobot 280** robotic arm (M5/Arduino version) from Elephant Robotics.

The project integrates **MoveIt 2** for motion planning and a **custom Python driver** that bridges ROS 2 with the real hardware through the `pymycobot` library, solving serial command saturation issues.

## Table of Contents

- [About the Robot](#about-the-robot)
- [Requirements](#requirements)
- [Docker Installation](#docker-installation)
- [Running](#running)
- [Current Limitations](#current-limitations)
- [Repository Structure](#repository-structure)
- [Resources](#resources)

---

## About the Robot

This project is designed for the **MyCobot 280**, a collaborative robot arm with **6 degrees of freedom (DoF)**.

| Property | Value |
|-----------|-------|
| Manufacturer | Elephant Robotics |
| Model | MyCobot 280 M5/Arduino |
| DoF | 6 |
| Official Repository | [mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2) |
| Documentation | [MyCobot Gitbook](https://docs.elephantrobotics.com/docs/mycobot280/) |

---

## Requirements

### Hardware
- **Host computer** running Linux
- **MyCobot 280** connected via USB (typically `/dev/ttyUSB0` or `/dev/ttyACM0`)
- USB device permissions

### Software
- **Docker** installed and configured
- Execution permissions (or sudo access)

---

## Docker Installation

> ℹ️ Docker is recommended to ensure all ROS 2 (Jazzy/Humble) dependencies and Python libraries are properly installed without affecting your host system.

### Step 1: Build the Image

```bash
docker build -t erobotics_image .
```

### Step 2: Run the Container

A helper script `run.sh` is included to automatically configure:
- Graphics permissions (for RViz)
- USB device mounting
- Required environment variables

```bash
# Grant execute permissions (first run)
chmod +x run.sh

# Run the container
./run.sh
```

You will land in a terminal inside the container, ready to run ROS 2 commands.

---

## Running

The system is divided into **two components** that must run in separate terminals (inside Docker):

1. **Driver**: Communicates with the hardware via serial port
2. **MoveIt + RViz**: Motion planning and visualization

### Terminal 1: Start the Hardware Driver

```bash
ros2 launch erobotics_driver driver.launch.py
```

**Description:**
- Connects to the robot via serial port
- Exposes the `FollowJointTrajectory` interface

**Expected success:** Log showing the detected port and "Driver Ready"

### Terminal 2: Start MoveIt and RViz

```bash
ros2 launch erobotics_moveit moveit.launch.py
```

**RViz usage:**

1. **Set target:** Drag the blue sphere ("Target") to the desired end-effector position
2. **Plan:** Click **"Plan"** (MoveIt computes the trajectory)
3. **Execute:** Click **"Execute"** (the real robot moves)

---

## Current Limitations

### Problem

The MyCobot 280 microcontroller has **bandwidth limitations** over serial communication. The robot cannot process dense trajectories (hundreds of points) at high speed without:
- Saturating
- Moving in jerky steps

### Implemented Solution: "Direct Mode"

The `erobotics_interface.py` controller implements a **Point-to-Point** strategy:

```
Full trajectory (MoveIt)
         ↓
  Ignore intermediate points
         ↓
  Extract final point (Goal)
         ↓
  Send a single command to the robot
```

**Advantages:**
- ✅ **Smooth motion:** The robot manages its own internal acceleration
- ✅ **Joint-space linearity:** All motors move simultaneously

**Limitations:**
- ⚠️ **Does not follow planned path:** All motors move at once (ignores MoveIt curve)
- ⚠️ **Does not avoid obstacles during motion:** Only considers start and end points
- ✅ **Ideal for:** Simple "Pick and Place" tasks without intermediate obstacles

### Recommended Use Cases

| Application | Recommended | Reason |
|-----------|:---:|--------|
| Simple Pick & Place | ✅ | Direct A → B motion |
| Tasks with obstacles | ❌ | Ignores planned curve |
| Complex trajectories | ❌ | Saturates serial port |

---

## Repository Structure

```
erobotics_ws/
├── src/
│   ├── erobotics_driver/          # Driver personalizado Python ↔ ROS 2
│   │   └── erobotics_interface.py # Puente con pymycobot
│   ├── erobotics_description/     # URDF/Xacro + mallas 3D
│   ├── erobotics_moveit/          # Configuración MoveIt Setup Assistant
│   └── erobotics_controller/      # Configuraciones de controladores ROS 2
├── build/                         # Build artifacts (colcon)
├── install/                       # Installed files
├── log/                           # Build logs
├── Dockerfile                     # Docker image definition
├── run.sh                         # Docker run script
└── README.md                      # This file
```

### Main Packages

| Package | Description |
|---------|-------------|
| `erobotics_driver` | Python script bridging ROS 2 and `pymycobot` |
| `erobotics_description` | URDF/Xacro files and 3D meshes of the robot |
| `erobotics_moveit` | Configuration generated by MoveIt Setup Assistant |
| `erobotics_controller` | Additional ROS 2 controller configurations |

---

## Resources

### Useful Links

- 📚 [Official MyCobot Documentation](https://docs.elephantrobotics.com/docs/mycobot280/)
- 🔗 [mycobot_ros2 Repository](https://github.com/elephantrobotics/mycobot_ros2)
- 📦 [pymycobot on PyPI](https://pypi.org/project/pymycobot/)
- 🎯 [MoveIt 2 Documentation](https://moveit.ros.org/documentation/getting_started/)

### Troubleshooting

**Robot won’t connect:**
- Verify the USB device is at `/dev/ttyUSB0` or `/dev/ttyACM0`
- Check permissions: `ls -la /dev/ttyUSB0`
- If using Docker, verify `run.sh` mounts the device correctly

**RViz does not appear:**
- Make sure you ran `./run.sh` (configures graphics permissions)
- Verify environment variables: `echo $DISPLAY`

---

> **Last update:** January 2026  
> **License:** Check the `LICENSE` file for more information
