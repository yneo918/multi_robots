# Multirobot Cluster Control Base Station

## Overview

This folder contains the base station packages to perform cluster control and adaptive navigation on a multi-robot platform. While this code was designed for the pioneer robots, it is mostly generalized and can be used for any multi-robot platform with similar DOF and could be extended further with slight modification.

### Key Features

1. **Cluster Control**: Implementation of cluster control algorithm
2. **Adaptive Navigation**: Implementation of adaptive navigation for scalar fields
3. **Manual Control**: Manual control during runtime of robots, mode selection, and cluster parameters using gamepad controller and GUI
4. **Configuration Files**: Change parameters more extensively before launch by editing configuration files
5. **Comprehensive Launch Files**: A nested series of launch files to launch different parts or the whole project

## File Structure
The following shows the file structure of the pioneer_base with the most important files labeled with comments.
```
~/ros2_ws/pioneer_base/
├── adaptive_nav/ 
│   ├── adaptive_nav.py               # Adaptive nav ROS manager
│   └── ScalarGradient.py             # Gradient Descent Class Definition
│   ├── launch/
|   │   ├── ANSim.launch.py           # full launch file for adaptive navigation in simulation
│   │   └── ANHardware.launch.py      # full launch file for adaptive navigation of physical robots
├── base_launch/                    
│   ├── launch/
|   │   ├── 5cluster_hw_with_desired.launch.py          # 5 robot cluster control full launch file
│   │   └── cluster_hw_with_desired.launch.py           # 3 robot cluster control full launch file
├── cluster_node/                       
│   └── Cluster.py                    # Cluster Controller Class definition
│   ├── config/                       # Folder for cluster configuration files
├── controller/                       # GPS sensor management
│   └── Controller.py                 # Cluster Controller ROS manager
├── lib/                       
│   └── my_ros_module.py              # Helper class for managing ros topics
├── sensor_receiver/                       
│   └── gps_viewer.py                 # Realtime plotting of robot GPS positions for testing
├── teleop_core/                      # This package contains all the files for user input through the GUI and jostick
│   ├── constants.py                  # Set joystick constants
│   ├── demux.py                      # Parses joystick commands to send velocity commands to robots in manual mode
│   ├── joy_cmd_base.py               # Parses gamepad controller input from usb port
│   ├── gui_base.py                   # GUI class definition
│   ├── run_joy_with_gui.py           # Full node for up to 5 robots containing GUI, joystick input, and demux
│   └── run_joy_with_gui3.py          # Full node for up to 3 robots containing GUI, joystick input, and demux
│   ├── launch/
|   │   ├── gui.launch.py           # full launch file for teleop_core with up to 5 robots
│   │   └── gui_3.launch.py         # full launch file for teleop_core with up to 3 robots
│   ├── config/                     # Folder for joystick configuration files
```

## Prerequisites

- **ROS 2 Jazzy** installed
- **Python 3.8+**
- **Required Python packages**:
  - `numpy'
  - `sympy'
- Rviz2 and xacro up to date to run simulation

## Quick Start

### 1. Installation

```bash
# Navigate to workspace
cd ~/ros2_ws/

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

### 2. Configuration
Many launch files pass in yaml files as parameters as such:
```py
# Get share directory of package
cluster_package_name = 'cluster_node'
cluster_pkg_share = get_package_share_directory(cluster_package_name)
# Construct paths to the parameter file within the shared package
cluster_file = os.path.join(cluster_pkg_share, 'config', '3cluster_velocity.yaml')
Node(
    package="controller",
    executable="cluster_controller",
    parameters=[cluster_file],        # pass in yaml file as parameter to node
)
```

You can edit the yaml files to modify the node parameters or create your own following the same format.

### Cluster configuration
3cluster_velocity.yaml contains an example of the cluster configuration
```yaml
cluster_feedback:                          # node name. Do not change unless you change node names in launch file
  ros__parameters:
    robot_id_list: ["p1", "p2", "p3"]      # list of robot ids typically p1-p5
    cluster_size: 3
    cluster_params: [8.0, 8.0, 1.047]      # initial cluster parameters base on cluster configuration. Look through Cluster.py for options.
    adaptive_navigation: True              # whether you intend to use adaptive navigation
    cluster_type: "TriangleatCentroid"     # name of cluster configuration from list in Cluster.py
    control_mode: "VEL"                    # whether you want to use velocity ("VEL") or postion ("POS") control for cluster
```
### Joystick configuration
joy-assign.yaml contains an example of the joystick mappings.
```yaml
joy_cmd:
  ros__parameters:
    # Movement controls
    lx: "LX"          # Forward/backward velocity
    ly: "LY"          # Left/right velocity (for holonomic drive)
    az: "RX"          # Angular velocity
    
    # Control buttons
    en: "LB"                    # Enable movement
    rover_sel: "Y"              # Select rover
    mode_sel: "X"               # Switch operation mode
    angle_sel: "A"              # Select angle
    hardware_sim_sel: "B"       # Hardware/simulation toggle
    broadcast: "RB"             # Broadcast mode
    
    # Rover configuration
    n_rover: 5
```
demux.yaml contains the list of robot ids you can control in manual mode.
```yaml
demux:
  ros__parameters:
    robot_id_list: ["p1", "p2", "p3", "p4", "p5"]
```
