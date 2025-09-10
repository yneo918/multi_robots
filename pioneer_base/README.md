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
