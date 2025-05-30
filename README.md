# Pioneer_base
For base station
# Pioneer_ws
For Pioneer Robots  
user name: pioneer-\#  
\# is i/ii/iii/iv/v/...

When you set-up RPi, Please use script in pioneer_ws/scripts. Usage is written in README.

# Diagrams:

# How to use
## Hardware test
With 3 Robot Cluster, please run follow:
```
ros2 launch base_launch cluster_hw_with_desired.launch.py
```

With 5 Robot Cluster, please run follow:
```
ros2 launch base_launch 5cluster_hw_with_desired.launch.py
```

Without HW joypad, please run follows too.
``` 
ros2 run virtual_joy virtual_joy
```

With cluster mode, please change mode to NAV_M, and press LB button.