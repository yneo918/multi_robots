# Simulator
## Install
```
pip install PyQt6
```
## How to use

### 3 Rover with Desiredposition
Display+simulation+cluster_controller:
```
ros2 launch sim_launch pioneer_with_desired.launch.py
```

### Display and simulation
```
ros2 launch rover_description sim.launch.py
```
### 3 Rover with HW
```
ros2 launch rover_description with_ghost.launch.py [params]
```

### Rover
One per unit to be activated.
```
ros2 launch rover_description pioneer.launch.py [params]
```
params:  
robot_id : string  
x, y, t : double

ex)  
```
ros2 launch rover_description pioneer.launch.py robot_id:="p2" x:=1.0 y:=1.0 t:=0.0
```
Then, you change below:  
RobotModel>Description Topic to /{robot_id}/robot_description  
RobotModel>TF Prefix to {robot_id}
