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
Please run follow:
```
ros2 launch base_launch cluster_hw_with_desired.launch.py
```

Without HW joypad, please run follows too.
``` 
ros2 run virtual_joy virtual_joy
```

If you need "fake" actual rover, please run folows:
```
ros2 launch sim_launch fake_pioneer.launch.py
```

With cluster mode, please change mode to NAV_M, and press LB button.