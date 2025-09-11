# Multirobot Simulation Environment

## Overview

The simulation environment is used for quick testing of algorithms and provides a visual representation of robots state to make it easier to analyze and debug. The simulation environment uses Rviz2 the Ros2 standard packages associated with it. Control of the simulation robots follows the same structure as the physical robots taking in a velocity command as input and outputing its pose in (x, y, 0). The simulation robots assume an ideal response to the velocity command and update there pose accordingly. 

## File Structure
The following shows the file structure of sim with the most important files labeled with comments.
```
~/ros2_ws/sim/
├── fake_rover_state_controller/ 
│   ├── fake_rover.py                 # Reads velocity commands and updates and publishes state for 1 rover
│   └── jointstate_publisher.py       # Collects and publishes additional states of rover such as desired position
├── rf_sim/ 
│   └── rf_field.py                   # Simulates an rf field for the purposes of adaptive navigation in simulation
├── rf_sim_interfaces/
│   ├── srv/
│   │   └── GetRXPower.srv            # Custom ros service used by rf_sim
├── rover_description/
│   ├── launch/
|   │   ├── ANSim.launch.py           # full launch file for adaptive navigation in simulation
│   │   └── ANHardware.launch.py      # full launch file for adaptive navigation of physical robots
│   ├── meshes/                       # meshes of pioneer robots to be shown in sim
│   ├── rviz/                         # Rviz files used when launching rviz to setup objects
│   └── src/description               # Urdf and xacro files to define joint connections that simulate rover movement
```
