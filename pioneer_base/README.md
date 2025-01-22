ros2_ws/src for pioneer base  
https://github.com/yneo918/pioneer_base.git  


## Usage
### Run Joypad node for multi pioneers
```
$ ros2 launch teleop_core teleop_node_multi.launch.py
```
You can set robot id in config/demux.yaml.

### Run Autonavigation(Under Dev)
```
$ ros2 run auto_nav_core nav_controller --ros-args --params-file src/auto_nav_core/config/navparams.yaml
```

### Joy Stick
```
LY: ↕  
RX: ↔   
LB: enable
```
You can set button assignment in config/joy-assign.yaml.
