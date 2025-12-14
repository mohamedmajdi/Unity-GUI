# Unity-based GUI

A Unity-based GUI application for real-time visualization and monitoring of underwater robots data from ROS2 & Stonefish/Unity simulations.

## Features

### ROS2 Integration
- **Odometry Subscriber** - Real-time position and orientation data from ROS2 topics
- **Topic Visualization** - Monitor and visualize multiple ROS2 topics:
  - Camera feed visualization
  - Velocity commands
  - Sensor data streams
  - Custom topic monitoring

### 3D Visualization
- **Real-time Robot Pose** - 3D representation of the robot position and orientation by applying received velocity commands to GUI robot for accurate motion representation
- **Odometry-driven Animation** - Robot model moves based on received odometry data
- **Interactive 3D View** - Pan, zoom, and rotate camera for different viewing angles

## Prerequisites

### Required Unity Packages

1. **ROS TCP Connector** - Communication bridge between Unity and ROS2
   - Repository: [https://github.com/Unity-Technologies/ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
   

2. **ROS Visualizations** - Built-in visualization tools for ROS messages
   - Repository: [https://github.com/Unity-Technologies/ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
  

### Required ROS2 Packages

**ROS TCP Endpoint** - Server-side ROS2 node for Unity communication
- Repository: [https://github.com/Unity-Technologies/ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)


## Setup

### Unity Configuration

1. **Configure ROS Settings**
   - Go to `Robotics > ROS Settings` in Unity
   - Set `ROS IP Address` to your ROS2 machine IP (use `127.0.0.1` for local)
   - Set `ROS Port` to `10000` (default)
   - Protocol: `ROS2`

2. **Configure Coordinate System**
   - Select the `BlueRov Velo Control (Script)` component linked to the robot
   - **Coordinate System Setting:**
     - **Use Unity Coordinates** - Check this when using **Unity Simulation Server**
     - ☐ **Use Unity Coordinates** - Uncheck this when using **Stonefish Simulation** or other simulations

### ROS2 Configuration

Ensure your Stonefish simulation or real robot is publishing the required topics names or do remapping for the topics to match the ones in Unity:


## Usage

### Step 1: Start ROS TCP Endpoint Server

Launch the ROS TCP endpoint to enable Unity-ROS2 communication:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10001
```

**Parameters:**
- `ROS_IP`: IP address of the machine running ROS2 (use `127.0.0.1` for local, or your machine's IP for network)
- `ROS_TCP_PORT`: TCP port for communication (port number should be different than the one used by another opened unity if any)

### Step 2: Launch Stonefish Simulation or Real Robot

### Step 3: Run Unity GUI

Open the Unity project and press Play to start the visualization GUI.

### Step 4: Monitor Robot in GUI

The GUI will automatically:
- Subscribe to odometry data
- Update 3D robot pose in real-time
- Visualize camera feed
- Display velocity commands
- Show topic data streams to be able to choose from them

