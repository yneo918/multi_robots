# Simulator
## How to use
### Display
```
ros2 launch rover_description display_with_sim.launch.py
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
