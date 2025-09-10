# Pioneer Multi-Robot Testbed

The pioneer multi-robot testbed is platform designed for development and testing of cluster control and adaptive navigation algorithms of land rovers. It can be deployed in a simulation environement or with physical robots with the ability to manually control the robots and change parameters during runtime.

---

## Project Overview

- **Motivation**: To collect extensive field data of cluster control and adaptive navigation algorithms of various cluster configurations.
- **Capabilities**:
  - Implementation of Cluster formation control and Adaptive navigation algorithms.
  - A simulation environment to allow for easy testing of algorithms during development and have a visual representation.
  - Robust pioneer rover platform with automatic script exectution, error handling, and status monitoring.
  - Use configuration files to switch between different cluster configurations from 3-5 robots.
  - User may switch between all modes, modify parameters, and take manual control of robots during runtime.

---

## Demo
Video of 3 robot cluster following contour clockwise in simulation.
<p align="center">
  <img src="record/simmr.gif" width="600" alt="an sim demo"/>
</p>
Video of 5 robot cluster moving forward in field.
<p align="center">
  <img src="record/5robot.gif" width="600" alt="5 robot"/>
</p>
---

## File Structure
Each of the highest level files has an extended read me detailed there purpose and use.
```
~/ros2_ws/
├── Docker/                # Files to launch project in docker container
├── common/                # Custom ROS types and utility classes used throughout 
├── pioneer_base/          # Base station often run on laptop
├── pioneer_ws/            # Pioneer rover platform
├── record/                # Logging and graphing data
└── sim/                   # Simulation environment
```

