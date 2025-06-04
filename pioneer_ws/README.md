# Pioneer ROS System - Enhanced Version Deployment Guide

## Overview

This enhanced version addresses the following issues:

1. **Stabilized Reset Commands**: Multiple transmission and retry functionality
2. **Sensor Node Stabilization**: Automatic reconnection and enhanced error handling
3. **Node Monitoring and Auto-Recovery**: Health monitoring system
4. **Centralized Configuration**: Manage all settings with a single configuration file

## New File Structure

```
pioneer_ws/
├── config/
│   └── system_config.yaml          # Unified configuration file (NEW)
├── scripts/
│   ├── setup.sh                    # Enhanced setup script
│   ├── start_ros2.sh              # Auto-generated startup script
│   ├── reset_sensors.py           # Sensor reset script (NEW)
│   └── health_monitor.py          # Health monitoring (NEW)
├── convert_pose/
│   └── convert_pose/
│       └── converter.py           # Enhanced version
├── gps_core/
│   └── gps_core/
│       └── gps_revised.py         # Enhanced version
└── imu_core/
    └── imu_core/
        └── imu_node.py             # Enhanced version
```

## Prerequisites

- ROS 2 Jazzy installed
- Python 3.8+
- Required Python packages: `yaml`, `psutil`, `adafruit-circuitpython-gps`, `adafruit-circuitpython-bno055`
- Sudo access for systemd service configuration

## Deployment Procedure

### 1. Create Configuration File

Create a configuration file for each robot:

```bash
# Create config directory
mkdir -p ~/ros2_ws/pioneer_ws/config

# Copy configuration file (from provided sample)
cp system_config.yaml ~/ros2_ws/pioneer_ws/config/
```

Edit the configuration file `~/ros2_ws/pioneer_ws/config/system_config.yaml`:

```yaml
# Robot identification
robot:
  id: "p2"  # ← Change robot ID

# User configuration  
user:
  name: "pioneer-ii"     # ← Change username
  home_dir: "/home/pioneer-ii"  # ← Change home directory
```

### 2. Deploy Enhanced Files

Place the provided enhanced files:

```bash
cd ~/ros2_ws/pioneer_ws

# Deploy Python files
cp converter.py convert_pose/convert_pose/
cp gps_revised.py gps_core/gps_core/
cp imu_node.py imu_core/imu_core/

# Deploy script files
cp setup.sh scripts/
cp reset_sensors.py scripts/
cp health_monitor.py scripts/

# Set execute permissions
chmod +x scripts/*.sh scripts/*.py
```

### 3. Run Setup

Execute the enhanced setup script:

```bash
cd ~/ros2_ws/pioneer_ws/scripts
./setup.sh
```

This script automatically performs:
- Configuration file reading
- Environment variable setup
- Startup script generation
- systemd service creation
- Workspace building

### 4. Verify Service

```bash
# Check service status
sudo systemctl status ros2_start.service

# View logs
journalctl -u ros2_start.service -f
```

## New Features Usage

### Sensor Reset Commands

Stable reset functionality:

```bash
# Reset both sensors
~/ros2_ws/pioneer_ws/scripts/reset_sensors.py --sensor both

# Reset GPS only
~/ros2_ws/pioneer_ws/scripts/reset_sensors.py --sensor gps

# Reset IMU only
~/ros2_ws/pioneer_ws/scripts/reset_sensors.py --sensor imu

# Interactive mode
~/ros2_ws/pioneer_ws/scripts/reset_sensors.py --interactive

# With retry count
~/ros2_ws/pioneer_ws/scripts/reset_sensors.py --sensor both --retries 5
```

### Health Monitoring

System monitoring and auto-recovery:

```bash
# Start health monitor
~/ros2_ws/pioneer_ws/scripts/health_monitor.py

# Start in daemon mode
~/ros2_ws/pioneer_ws/scripts/health_monitor.py --daemon

# With configuration file
~/ros2_ws/pioneer_ws/scripts/health_monitor.py --config ~/ros2_ws/pioneer_ws/config/system_config.yaml

# Verbose logging
~/ros2_ws/pioneer_ws/scripts/health_monitor.py --verbose

# Specific robot ID
~/ros2_ws/pioneer_ws/scripts/health_monitor.py --robot-id p3
```

### Health Monitor Features

- **Real-time monitoring**: GPS, IMU, and converter node health
- **Automatic restart**: Failed nodes are automatically restarted
- **Data rate monitoring**: Tracks data rates for each sensor
- **Error counting**: Accumulates and tracks errors
- **Automatic reset**: Sends reset commands when error threshold is exceeded
- **System statistics**: Uptime, restart counts, and resource usage

## Multi-Robot Operation

### Setup for New Robot

1. **Copy configuration file**:
   ```bash
   cp system_config.yaml system_config_p3.yaml
   ```

2. **Edit configuration**:
   ```yaml
   robot:
     id: "p3"  # New robot ID
   user:
     name: "pioneer-iii"     # New username
     home_dir: "/home/pioneer-iii"
   ```

3. **Run setup**:
   ```bash
   cp system_config_p3.yaml system_config.yaml
   ./setup.sh
   ```

### Centralized Management (Optional)

Manage all robot configurations from one location:

```bash
# Configuration management directory
mkdir -p ~/robot_configs

# Individual robot configurations
~/robot_configs/
├── p1_config.yaml
├── p2_config.yaml
└── p3_config.yaml
```

### Deployment Script for Multiple Robots

```bash
#!/bin/bash
# deploy_to_robot.sh

ROBOT_ID=$1
CONFIG_FILE="~/robot_configs/${ROBOT_ID}_config.yaml"

if [ -z "$ROBOT_ID" ]; then
    echo "Usage: $0 <robot_id>"
    exit 1
fi

echo "Deploying to robot $ROBOT_ID..."

# Copy config and run setup
scp $CONFIG_FILE ${ROBOT_ID}@robot-${ROBOT_ID}:~/ros2_ws/pioneer_ws/config/system_config.yaml
ssh ${ROBOT_ID}@robot-${ROBOT_ID} "cd ~/ros2_ws/pioneer_ws/scripts && ./setup.sh"
```

## Troubleshooting

### 1. Sensor Initialization Failure

```bash
# Check health monitor logs
journalctl -u ros2_start.service -f | grep -E "(GPS|IMU)"

# Manual sensor reset
~/ros2_ws/pioneer_ws/scripts/reset_sensors.py --sensor both

# Check device connections
ls -la /dev/tty*
```

### 2. Node Crashes

```bash
# Health monitor automatically attempts recovery
# Check logs for restart attempts
journalctl -u ros2_start.service -f | grep "restart"

# Manual node restart
ros2 run gps_core gps_revised  # GPS
ros2 run imu_core run_imu      # IMU
ros2 run convert_pose converter # Converter

# Check node status
ros2 node list | grep p2  # Replace p2 with your robot ID
```

### 3. Configuration Issues

```bash
# Verify current configuration
cat ~/ros2_ws/pioneer_ws/config/system_config.yaml

# Check environment variables
echo $ROBOT_ID
env | grep ROBOT

# Validate YAML syntax
python3 -c "import yaml; yaml.safe_load(open('~/ros2_ws/pioneer_ws/config/system_config.yaml'))"
```

### 4. Service Issues

```bash
# Check service status
sudo systemctl status ros2_start.service

# Restart service
sudo systemctl restart ros2_start.service

# View service configuration
cat /etc/systemd/system/ros2_start.service

# Check service logs
journalctl -u ros2_start.service --since "1 hour ago"
```

### 5. Complete System Reset

```bash
# Stop service
sudo systemctl stop ros2_start.service

# Clean build
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install

# Re-run setup
cd pioneer_ws/scripts
./setup.sh --clean

# Restart service
sudo systemctl start ros2_start.service
```

## Health Monitor Output Examples

### Normal Operation
```
[INFO] Health monitor started for p2
[INFO] Monitoring interval: 5.0s
[INFO] Health Status - gps: HEALTHY (restarts: 0, rate: 10.2Hz, errors: 0), 
       imu: HEALTHY (restarts: 0, rate: 9.8Hz, errors: 0), 
       converter: HEALTHY (restarts: 0, rate: 10.1Hz, errors: 0)
```

### Node Recovery
```
[WARN] GPS data timeout detected
[WARN] Node gps is unhealthy, attempting restart
[INFO] Restarting gps node...
[INFO] Node gps restarted with PID 12345
```

### System Statistics
```
[INFO] System Statistics - Uptime: 2.5h, Total Restarts: 1, 
       GPS: OK (10.2Hz), IMU: OK (9.8Hz), Converter: OK (10.1Hz)
```

## Enhanced Features Details

### 1. Robust Error Handling
- Try-catch blocks added to all nodes
- Automatic retry on connection failures
- Thread-safe data access

### 2. Communication Stabilization
- Multiple transmission of reset commands
- Improved timeout handling
- Exception handling in callback functions

### 3. Health Monitoring System
- Sensor data monitoring
- Automatic recovery on anomaly detection
- Enhanced logging

### 4. Simplified Configuration Management
- Single configuration file
- Automatic script generation
- Automatic environment variable setup

## Performance Monitoring

### Key Metrics to Monitor

1. **Data Rates**:
   - GPS: ~10Hz (expected)
   - IMU: ~10Hz (expected)
   - Converter: ~10Hz (expected)

2. **Error Counts**:
   - Should remain low (<5 per hour)
   - High error counts indicate hardware issues

3. **Restart Frequency**:
   - Occasional restarts are normal
   - Frequent restarts (>3 per hour) indicate systematic issues

4. **System Resources**:
   - CPU usage should be <50%
   - Memory usage should be <80%
   - Available disk space >1GB

### Monitoring Commands

```bash
# Real-time health status
ros2 topic echo /p2/health_status

# System resource usage
htop
df -h
free -h

# Network connectivity
ping basestation-ip

# ROS node status
ros2 node list
ros2 topic list
```

This enhanced system provides significantly improved stability and easier management across multiple robots. The automatic recovery features reduce manual intervention requirements, while the centralized configuration simplifies deployment and maintenance.
