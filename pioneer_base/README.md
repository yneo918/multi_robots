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

## Message
```
Role:          msg_name                    Type
joy_input:     /joy                        sensor_msgs.Joy
broadcast:     /joy/broadcast              std_msgs.msg.Bool
joy_cmd_vel:   /joy/cmd_vel                geometry_msgs.msg.Twist
joy_enable:    /joy/enable                 std_msgs.msg.Bool
select_rover:  /select_rover               std_msgs.msg.Int16
cmd_vel_rover: /{ROBOT_ID}/cmd_vel         geometry_msgs.msg.Twist

```
