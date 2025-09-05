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

```
~/ros2_ws/pioneer_base/
├── adaptive_nav/
│   ├── adaptive_nav.py             # Adaptive nav ROS manager
│   ├── ScalarGradient.py           # Gradient Descent Class Definition
├── base_launch/                   # GPS/IMU to pose conversion
│   ├── launch/
│   │   └── cluster_hw_with_desired.launch.py           # 3 robot cluster control full launch file
├── cluster_node/                       
│   └── Cluster.py                 # Cluster Controller Class definition
├── controller/                       # GPS sensor management
│   └── Controller.py                 # Cluster Controller ROS manager
```
