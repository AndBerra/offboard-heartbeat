# Offboard Heartbeat

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Iron-blue)](https://docs.ros.org/en/rolling/)
[![PX4](https://img.shields.io/badge/PX4-SITL-orange)](https://px4.io/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)

A ROS 2 package that automatically switches PX4 drones to offboard mode for simulation environments.

## Overview

This package provides a simple ROS 2 node that automatically engages PX4's offboard mode during simulation startup. It's designed for automated testing workflows where manual mode switching would be inconvenient.

**Key Features:**
- 🚀 Automatic offboard mode engagement after 1-second delay
- 💓 Continuous heartbeat signal maintenance  
- 🔀 Multi-vehicle namespace support
- 🎯 Seamless PX4 SITL integration

## ⚠️ **Safety Warning**

**🚨 FOR SITL USE ONLY! 🚨**

This package **automatically forces** drones into offboard mode, removing all manual pilot control. 

Use only with PX4 SITL simulation. **Never use on real hardware**: risk of loss of control and crashes.

## How It Works

1. **Heartbeat Transmission**: Publishes offboard control signals at 10Hz
2. **Auto Mode Switch**: Engages offboard mode after 1-second initialization  
3. **Continuous Operation**: Maintains offboard state with position control enabled

## Quick Start

### Prerequisites
- ROS 2 
- PX4 SITL (tested on v1.16)
- `px4_msgs` package

### Installation
```bash
cd ~/ros2_ws/src
git clone https://github.com/AndBerra/offboard-heartbeat.git
cd ~/ros2_ws && colcon build --packages-select offboard_heartbeat
source install/setup.bash
```

### Usage

**Single Vehicle:**
```bash
ros2 launch offboard_heartbeat offboard_heartbeat.launch.py
```

**Multi-Vehicle (with namespace):**
```bash
ros2 launch offboard_heartbeat offboard_heartbeat.launch.py namespace:=px4_1
```

**With PX4 SITL:**
```bash
# Terminal 1: Start PX4 SITL
make px4_sitl gazebo-classic

# Terminal 2: Launch heartbeat (10s delay included)
ros2 launch offboard_heartbeat offboard_heartbeat.launch.py
```

### Parameters
- `namespace`: Topic namespace for multi-vehicle setups (default: empty)

## License

Apache 2.0 License - see [LICENSE](LICENSE) file.

## Maintainer

**Andrea Berra** - [andrea.berra@outlook.com](mailto:andrea.berra@outlook.com)