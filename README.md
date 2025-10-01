# Offboard Heartbeat

A ROS 2 package for maintaining PX4 autopilot in offboard control mode through continuous heartbeat signals.

## Overview

This package provides a ROS 2 node that enables and maintains PX4-based aerial vehicles (drones, VTOL, etc.) in offboard control mode. It's designed for robotics applications where external control systems need to command the vehicle through the PX4 flight stack.

## What Does This Package Do?

In PX4 autopilot systems, **offboard mode** allows external computers (companion computers) to send control commands to the vehicle. However, PX4 requires continuous heartbeat signals to remain in offboard mode as a safety mechanism. If these signals stop, the vehicle will exit offboard mode.

This package:
- **Automatically engages offboard mode** on the PX4 autopilot
- **Maintains offboard mode** by continuously sending heartbeat signals at 10Hz
- **Monitors vehicle status** to track the current flight mode
- **Supports multi-vehicle systems** through ROS 2 namespaces
- **Uses PX4-ROS 2 bridge** for reliable communication with the flight controller

**Use Case**: This node is essential when you want to control your PX4-based drone from a higher-level autonomy stack, motion planning system, or custom control algorithm running on a companion computer.

## Requirements

### Dependencies
- **ROS 2** (tested with Humble or newer)
- **Python 3.8+**
- **px4_msgs**: PX4 message definitions for ROS 2
- **PX4-ROS 2 bridge**: For communication between ROS 2 and PX4 autopilot

### Hardware/Simulation
- PX4 autopilot (hardware or SITL simulation)
- Running PX4-ROS 2 bridge (e.g., `MicroXRCEAgent`)

## Installation

1. **Navigate to your ROS 2 workspace**:
   ```bash
   cd ~/ros2_ws/src
   ```

2. **Clone the repository**:
   ```bash
   git clone https://github.com/AndBerra/offboard-heartbeat.git
   ```

3. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the package**:
   ```bash
   colcon build --packages-select offboard_heartbeat
   ```

5. **Source the workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Basic Launch

**Method 1: Using the launch file (recommended)**
```bash
ros2 launch offboard_heartbeat offboard_heartbeat.launch.py
```

The launch file includes a 10-second delay before starting the node, giving time for the PX4 system to initialize.

**Method 2: Running the node directly**
```bash
ros2 run offboard_heartbeat offboard_heartbeat_node
```

### Multi-Vehicle Support (with Namespaces)

For multi-vehicle systems where each vehicle uses a separate namespace:

```bash
ros2 launch offboard_heartbeat offboard_heartbeat.launch.py namespace:=uav1
```

Or directly:
```bash
ros2 run offboard_heartbeat offboard_heartbeat_node --namespace uav1
```

## Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | Heartbeat signal indicating offboard control is active (position control mode) |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | Commands sent to the vehicle (e.g., mode change to offboard) |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | Current vehicle status and flight mode information |

**Note**: When using a namespace (e.g., `uav1`), topics are prefixed: `/uav1/fmu/in/offboard_control_mode`

## Parameters

### Launch Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `namespace` | string | `""` | ROS 2 namespace for multi-vehicle support. Empty string uses default topics. |

## How It Works

### Operational Sequence

1. **Initialization** (0-1 seconds):
   - Node starts and configures QoS profiles for PX4 communication
   - Establishes publishers and subscribers
   - Begins 10Hz timer loop

2. **Heartbeat Phase** (1-2 seconds):
   - Sends continuous offboard control mode messages
   - Publishes heartbeat signals indicating position control mode
   - Counter increments with each cycle

3. **Mode Engagement** (at ~1 second):
   - After 10 heartbeat cycles, sends mode change command
   - Triggers transition to offboard mode (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED + PX4_CUSTOM_MAIN_MODE_OFFBOARD)

4. **Sustained Operation** (2+ seconds):
   - Continues sending heartbeat signals indefinitely
   - Maintains offboard mode as long as node is running
   - Monitors vehicle status

### Control Mode Configuration

The node configures offboard control for **position control**:
- `position = True` - Position control enabled
- `velocity = False` - Velocity control disabled
- `acceleration = False` - Acceleration control disabled
- `attitude = False` - Attitude control disabled
- `body_rate = False` - Body rate control disabled

**Important**: This node only enables and maintains offboard mode. Actual setpoint commands (position, velocity, etc.) must be sent by your control application.

## Architecture

### Node Structure

```
OffboardHeartBeat Node
├── Publishers
│   ├── offboard_control_mode_publisher (10Hz)
│   └── vehicle_command_publisher (on demand)
├── Subscribers
│   └── vehicle_status_subscriber
└── Timer (0.1s / 10Hz)
    └── timer_callback()
        ├── publish_offboard_control_heartbeat_signal()
        └── engage_offboard_mode() [once at counter=10]
```

### QoS Profile

Uses PX4-compatible QoS settings:
- **Reliability**: Best Effort
- **Durability**: Transient Local
- **History**: Keep Last (depth=1)

These settings ensure compatibility with the PX4-ROS 2 bridge and minimize latency.

## Integration with Your Robot

### Typical System Architecture

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────┐
│   Your Control  │────────▶│  Offboard        │────────▶│  PX4 Flight │
│   Application   │         │  Heartbeat Node  │         │  Controller │
│  (Position/Vel  │         │  (This Package)  │         │             │
│   Commands)     │         │                  │         │             │
└─────────────────┘         └──────────────────┘         └─────────────┘
        │                           │                           │
        │                           │                           │
        └───────────────────────────┴───────────────────────────┘
                        ROS 2 Topics (via PX4-ROS 2 Bridge)
```

### Workflow
1. Start PX4 (hardware or SITL)
2. Start PX4-ROS 2 bridge (`MicroXRCEAgent`)
3. Launch this offboard heartbeat node
4. Send your control commands (position, velocity, etc.) from your application
5. The vehicle executes your commands while heartbeat maintains offboard mode

## Example: Complete Setup with PX4 SITL

```bash
# Terminal 1: Start PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2: Start MicroXRCE Agent
MicroXRCEAgent udp4 -p 8888

# Terminal 3: Launch offboard heartbeat
source ~/ros2_ws/install/setup.bash
ros2 launch offboard_heartbeat offboard_heartbeat.launch.py

# Terminal 4: Send your control commands
ros2 topic pub /fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint ...
```

## Safety Considerations

- **Always test in simulation first** (e.g., PX4 SITL with Gazebo)
- This node will engage offboard mode automatically - ensure your control system is ready
- Have a manual override method (RC controller) available
- The vehicle will exit offboard mode if:
  - This node stops running
  - Heartbeat signals are interrupted
  - RC override is triggered
  - Emergency stop is activated

## Troubleshooting

### Node starts but vehicle doesn't enter offboard mode
- Verify PX4-ROS 2 bridge is running and connected
- Check that PX4 is armed (vehicle must be armed to enter offboard mode)
- Ensure topics are correctly mapped (check with `ros2 topic list`)

### "QoS mismatch" warnings
- This is normal with PX4 topics due to QoS profile differences
- As long as messages are being received/sent, operation is correct

### Multi-vehicle namespace not working
- Ensure the bridge is configured with the correct namespace
- Verify topic remapping in the bridge configuration

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Maintainer

- **Andrea Berra** - [andrea.berra@outlook.com](mailto:andrea.berra@outlook.com)

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## Additional Resources

- [PX4 Offboard Control Documentation](https://docs.px4.io/main/en/flight_modes/offboard.html)
- [PX4-ROS 2 Bridge](https://docs.px4.io/main/en/ros/ros2_comm.html)
- [px4_msgs Repository](https://github.com/PX4/px4_msgs)

## Version

**Current Version**: 0.0.1
