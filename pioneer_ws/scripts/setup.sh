#!/bin/bash

# Enhanced setup script for Pioneer ROS system with full configuration support
# This script configures the system using ALL settings from the central config file

set -e  # Exit on any error

CONFIG_FILE="config/system_config.yaml"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Function to read YAML values
read_yaml() {
    local file=$1
    local key=$2
    python3 -c "
import yaml
import sys
try:
    with open('$file', 'r') as f:
        data = yaml.safe_load(f)
    keys = '$key'.split('.')
    value = data
    for k in keys:
        value = value[k]
    print(value)
except Exception as e:
    print('', file=sys.stderr)
    sys.exit(1)
"
}

# Check if config file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Configuration file $CONFIG_FILE not found"
    echo "Please create the configuration file first"
    exit 1
fi

# Read all configuration values
ROBOT_ID=$(read_yaml "$CONFIG_FILE" "robot.id")
USER_NAME=$(read_yaml "$CONFIG_FILE" "user.name")
HOME_DIR=$(read_yaml "$CONFIG_FILE" "user.home_dir")
WORK_DIR="$HOME_DIR/ros2_ws"

# Read GPS configuration
GPS_BAUDRATE=$(read_yaml "$CONFIG_FILE" "gps.baudrate")
GPS_TIMEOUT=$(read_yaml "$CONFIG_FILE" "gps.timeout")
GPS_CONNECTION_TIMEOUT=$(read_yaml "$CONFIG_FILE" "gps.connection_timeout")
GPS_RETRY_DELAY=$(read_yaml "$CONFIG_FILE" "gps.retry_delay")
GPS_MOVING_AVG_RATE=$(read_yaml "$CONFIG_FILE" "gps.moving_average_rate")
GPS_UPDATE_RATE=$(read_yaml "$CONFIG_FILE" "gps.update_rate" || echo "1.0")
GPS_TIMER_PERIOD=$(read_yaml "$CONFIG_FILE" "gps.timer_period" || echo "1.0")

# Read IMU configuration
IMU_HEADING_OFFSET=$(read_yaml "$CONFIG_FILE" "imu.heading_offset")
IMU_CONNECTION_TIMEOUT=$(read_yaml "$CONFIG_FILE" "imu.connection_timeout")
IMU_RETRY_DELAY=$(read_yaml "$CONFIG_FILE" "imu.retry_delay")
IMU_CALIBRATION_FILE=$(read_yaml "$CONFIG_FILE" "imu.calibration_file")

# Read Locomotion configuration
LOCOMOTION_MAX_VEL=$(read_yaml "$CONFIG_FILE" "locomotion.max_velocity")
LOCOMOTION_MAX_VEL_OPEN=$(read_yaml "$CONFIG_FILE" "locomotion.max_velocity_open")
LOCOMOTION_SERIAL_PORT=$(read_yaml "$CONFIG_FILE" "locomotion.serial_port")
LOCOMOTION_BAUDRATE=$(read_yaml "$CONFIG_FILE" "locomotion.baudrate")
LOCOMOTION_LEFT_MOTOR_SIGN=$(read_yaml "$CONFIG_FILE" "locomotion.left_motor_sign")
LOCOMOTION_RIGHT_MOTOR_SIGN=$(read_yaml "$CONFIG_FILE" "locomotion.right_motor_sign")

# Read Pose converter configuration
POSE_TIMER_PERIOD=$(read_yaml "$CONFIG_FILE" "pose_converter.timer_period")
POSE_HEALTH_CHECK_PERIOD=$(read_yaml "$CONFIG_FILE" "pose_converter.health_check_period")
POSE_RESET_TIMEOUT=$(read_yaml "$CONFIG_FILE" "pose_converter.reset_timeout")

# Read Monitoring configuration
MONITOR_SENSOR_TIMEOUT=$(read_yaml "$CONFIG_FILE" "monitoring.sensor_timeout")
MONITOR_MAX_RETRIES=$(read_yaml "$CONFIG_FILE" "monitoring.max_node_retries")
MONITOR_RESTART_DELAY=$(read_yaml "$CONFIG_FILE" "monitoring.node_restart_delay")

if [ -z "$ROBOT_ID" ] || [ -z "$USER_NAME" ] || [ -z "$HOME_DIR" ]; then
    echo "Error: Failed to read required configuration values"
    echo "Please check your configuration file: $CONFIG_FILE"
    exit 1
fi

echo "Setting up system for:"
echo "  Robot ID: $ROBOT_ID"
echo "  User: $USER_NAME"
echo "  Home Directory: $HOME_DIR"
echo "  Workspace: $WORK_DIR"
echo ""
echo "Configuration loaded:"
echo "  GPS: baudrate=$GPS_BAUDRATE, timeout=$GPS_TIMEOUT, update_rate=${GPS_UPDATE_RATE}Hz"
echo "  IMU: heading_offset=$IMU_HEADING_OFFSET"
echo "  Locomotion: max_vel=$LOCOMOTION_MAX_VEL"
echo "  Monitoring: sensor_timeout=$MONITOR_SENSOR_TIMEOUT"

# Create necessary directories
echo "Creating directories..."
mkdir -p "$HOME_DIR/imu_calib"
mkdir -p "$HOME_DIR/.ros"
mkdir -p "$WORK_DIR/pioneer_ws/config/nodes"

# Generate individual node parameter files from system config
echo "Generating node parameter files..."

# GPS node parameters
cat > "$WORK_DIR/pioneer_ws/config/nodes/gps_params.yaml" << EOF
# Auto-generated GPS parameters from system config
${ROBOT_ID}_gps1:
  ros__parameters:
    robot_id: "$ROBOT_ID"
    baudrate: $GPS_BAUDRATE
    timeout: $GPS_TIMEOUT
    connection_timeout: $GPS_CONNECTION_TIMEOUT
    retry_delay: $GPS_RETRY_DELAY
    moving_average_rate: $GPS_MOVING_AVG_RATE
    update_rate: $GPS_UPDATE_RATE
    timer_period: $GPS_TIMER_PERIOD
    device_paths: ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]
EOF

# IMU node parameters  
cat > "$WORK_DIR/pioneer_ws/config/nodes/imu_params.yaml" << EOF
# Auto-generated IMU parameters from system config
${ROBOT_ID}_imu:
  ros__parameters:
    robot_id: "$ROBOT_ID"
    heading_offset: $IMU_HEADING_OFFSET
    connection_timeout: $IMU_CONNECTION_TIMEOUT
    retry_delay: $IMU_RETRY_DELAY
    calibration_file: "$HOME_DIR/$IMU_CALIBRATION_FILE"
    calibFileLoc: "$HOME_DIR/$IMU_CALIBRATION_FILE"
EOF

# Pose converter parameters
cat > "$WORK_DIR/pioneer_ws/config/nodes/converter_params.yaml" << EOF
# Auto-generated converter parameters from system config
${ROBOT_ID}_pose_converter:
  ros__parameters:
    robot_id: "$ROBOT_ID"
    timer_period: $POSE_TIMER_PERIOD
    health_check_period: $POSE_HEALTH_CHECK_PERIOD
    reset_timeout: $POSE_RESET_TIMEOUT
EOF

# Locomotion parameters
cat > "$WORK_DIR/pioneer_ws/config/nodes/locomotion_params.yaml" << EOF
# Auto-generated locomotion parameters from system config
${ROBOT_ID}_movebase_kinematics:
  ros__parameters:
    robot_id: "$ROBOT_ID"
    max_vel: $LOCOMOTION_MAX_VEL
    max_vel_open: $LOCOMOTION_MAX_VEL_OPEN
    left_motor_sign: $LOCOMOTION_LEFT_MOTOR_SIGN
    right_motor_sign: $LOCOMOTION_RIGHT_MOTOR_SIGN
    
${ROBOT_ID}_cmd_roboteq:
  ros__parameters:
    robot_id: "$ROBOT_ID"
    serial_port: "$LOCOMOTION_SERIAL_PORT"
    baudrate: $LOCOMOTION_BAUDRATE
EOF

# Export environment variables for current session
export ROBOT_ID="$ROBOT_ID"
export IMU_OFFSET="$IMU_HEADING_OFFSET"

# Update .bashrc with environment variables
echo "Updating .bashrc..."
BASHRC="$HOME_DIR/.bashrc"

# Remove existing exports
sed -i '/export ROBOT_ID=/d' "$BASHRC" 2>/dev/null || true
sed -i '/export IMU_OFFSET=/d' "$BASHRC" 2>/dev/null || true

# Add new exports
echo "" >> "$BASHRC"
echo "# Pioneer ROS Environment Variables (auto-generated)" >> "$BASHRC"
echo "export ROBOT_ID=$ROBOT_ID" >> "$BASHRC"
echo "export IMU_OFFSET=$IMU_HEADING_OFFSET" >> "$BASHRC"

# Generate enhanced launch file that uses parameter files
LAUNCH_DIR="$WORK_DIR/pioneer_ws/rover_launch/launch"
echo "Generating enhanced launch files..."

cat > "$LAUNCH_DIR/rover_with_params.launch.py" << EOF
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    robot_id = os.getenv("ROBOT_ID", "$ROBOT_ID")
    config_dir = os.path.join(os.getenv("HOME"), "ros2_ws/pioneer_ws/config/nodes")
    
    ld = LaunchDescription()
    
    # GPS node with parameters
    gps_node = Node(
        package="gps_core",
        executable="run_gps1",
        parameters=[os.path.join(config_dir, "gps_params.yaml")],
        output='screen'
    )
    
    # IMU node with parameters
    imu_node = Node(
        package="imu_core",
        executable="run_imu",
        parameters=[os.path.join(config_dir, "imu_params.yaml")],
        output='screen'
    )
    
    # Locomotion nodes with parameters
    movebase_node = Node(
        package="locomotion_core",
        executable=f"movebase_kinematics",
        parameters=[os.path.join(config_dir, "locomotion_params.yaml")],
        output='screen'
    )
    
    cmd_roboteq_node = Node(
        package="locomotion_core",
        executable=f"cmd_roboteq",
        parameters=[os.path.join(config_dir, "locomotion_params.yaml")],
        output='screen'
    )
    
    # Converter node with parameters
    converter_node = Node(
        package="convert_pose",
        executable="converter",
        parameters=[os.path.join(config_dir, "converter_params.yaml")],
        output='screen'
    )
    
    ld.add_action(gps_node)
    ld.add_action(imu_node)
    ld.add_action(movebase_node)
    ld.add_action(cmd_roboteq_node)
    ld.add_action(converter_node)
    
    return ld
EOF

# Generate startup script
STARTUP_SCRIPT="$WORK_DIR/pioneer_ws/scripts/start_ros2.sh"
echo "Generating startup script: $STARTUP_SCRIPT"

cat > "$STARTUP_SCRIPT" << 'STARTUP_EOF'
#!/bin/bash

# Enhanced startup script with full configuration support
# Robot ID: ROBOT_ID_PLACEHOLDER
# User: USER_NAME_PLACEHOLDER

# Source ROS environment
source /opt/ros/jazzy/setup.bash
source WORK_DIR_PLACEHOLDER/install/setup.bash

# Set environment variables
export ROBOT_ID=ROBOT_ID_PLACEHOLDER
export IMU_OFFSET=IMU_OFFSET_PLACEHOLDER

# Change to workspace directory
cd WORK_DIR_PLACEHOLDER

# Function to verify parameter files exist
verify_param_files() {
    local param_dir="WORK_DIR_PLACEHOLDER/pioneer_ws/config/nodes"
    local required_files=("gps_params.yaml" "imu_params.yaml" "converter_params.yaml" "locomotion_params.yaml")
    
    for file in "${required_files[@]}"; do
        if [ ! -f "$param_dir/$file" ]; then
            echo "Error: Parameter file $file not found"
            echo "Run setup.sh to regenerate parameter files"
            return 1
        fi
    done
    return 0
}

# Function to start node with parameters and retry logic
start_node_with_params() {
    local package=$1
    local executable=$2
    local param_file=$3
    local max_retries=3
    local retry_delay=5
    
    for ((i=1; i<=max_retries; i++)); do
        echo "Starting $package/$executable with params from $param_file (attempt $i/$max_retries)"
        
        if ros2 run $package $executable --ros-args --params-file $param_file; then
            echo "$package/$executable started successfully"
            return 0
        else
            echo "$package/$executable failed to start"
            if [ $i -lt $max_retries ]; then
                echo "Retrying in $retry_delay seconds..."
                sleep $retry_delay
            fi
        fi
    done
    
    echo "Failed to start $package/$executable after $max_retries attempts"
    return 1
}

# Verify parameter files
if ! verify_param_files; then
    echo "Parameter files missing! Running setup to regenerate..."
    cd WORK_DIR_PLACEHOLDER/pioneer_ws/scripts
    ./setup.sh --regen-params-only
fi

# Start launch file with parameters
echo "Starting Pioneer ROS system with full configuration support..."
echo "Robot ID: $ROBOT_ID"
echo "Config directory: WORK_DIR_PLACEHOLDER/pioneer_ws/config/nodes"
echo "Timestamp: $(date)"

# Use the enhanced launch file with parameters
ros2 launch rover_launch rover_with_params.launch.py

STARTUP_EOF

# Replace placeholders in startup script
sed -i "s|ROBOT_ID_PLACEHOLDER|$ROBOT_ID|g" "$STARTUP_SCRIPT"
sed -i "s|USER_NAME_PLACEHOLDER|$USER_NAME|g" "$STARTUP_SCRIPT"
sed -i "s|WORK_DIR_PLACEHOLDER|$WORK_DIR|g" "$STARTUP_SCRIPT"
sed -i "s|IMU_OFFSET_PLACEHOLDER|$IMU_HEADING_OFFSET|g" "$STARTUP_SCRIPT"

chmod +x "$STARTUP_SCRIPT"

# Check which supplementary groups exist
echo "Checking system groups..."
SUPP_GROUPS=""
for group in dialout; do
    if getent group $group > /dev/null 2>&1; then
        echo "  Group '$group' exists"
        SUPP_GROUPS="$SUPP_GROUPS $group"
    else
        echo "  Group '$group' does not exist"
    fi
done

# Add user to necessary groups
echo "Adding user to necessary groups..."
for group in dialout; do
    if getent group $group > /dev/null 2>&1; then
        if ! groups $USER_NAME | grep -q $group; then
            echo "  Adding $USER_NAME to $group group"
            sudo usermod -a -G $group $USER_NAME
        else
            echo "  $USER_NAME is already in $group group"
        fi
    fi
done

# Generate systemd service file with enhanced configuration
SERVICE_NAME=$(read_yaml "$CONFIG_FILE" "service.name")
SERVICE_DESC=$(read_yaml "$CONFIG_FILE" "service.description")
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

echo "Generating systemd service file: $SERVICE_FILE"

# Build the service file content
SERVICE_CONTENT="[Unit]
Description=$SERVICE_DESC for $ROBOT_ID
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
Environment=\"ROBOT_ID=$ROBOT_ID\"
Environment=\"IMU_OFFSET=$IMU_HEADING_OFFSET\"
Environment=\"ROS_DOMAIN_ID=0\"
Environment=\"DISPLAY=:0\"
Environment=\"HOME=$HOME_DIR\"
User=$USER_NAME
Group=$USER_NAME
WorkingDirectory=$WORK_DIR
ExecStart=/bin/bash $STARTUP_SCRIPT
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal
KillMode=mixed
TimeoutStopSec=30"

# Only add SupplementaryGroups if there are groups to add
if [ -n "$SUPP_GROUPS" ]; then
    SERVICE_CONTENT="$SERVICE_CONTENT
SupplementaryGroups=$SUPP_GROUPS"
fi

SERVICE_CONTENT="$SERVICE_CONTENT

[Install]
WantedBy=multi-user.target"

# Write the service file
echo "$SERVICE_CONTENT" | sudo tee "$SERVICE_FILE" > /dev/null

# Handle --regen-params-only flag
if [ "$1" = "--regen-params-only" ]; then
    echo "Parameter files regenerated successfully"
    exit 0
fi

# Build the workspace
echo "Building workspace..."
cd "$WORK_DIR"

# Clean build if requested
if [ "$1" = "--clean" ]; then
    echo "Performing clean build..."
    rm -rf build install log
fi

# Build with error handling
if colcon build --symlink-install; then
    echo "Build completed successfully"
else
    echo "Build failed!"
    exit 1
fi

# Source the setup file
source install/setup.bash

# Enable and start the service
echo "Setting up systemd service..."
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}.service"

# Ask user if they want to start the service now
read -p "Do you want to start the service now? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo systemctl stop "${SERVICE_NAME}.service" 2>/dev/null || true
    sleep 2
    sudo systemctl start "${SERVICE_NAME}.service"
    sleep 2
    echo "Service status:"
    sudo systemctl status "${SERVICE_NAME}.service" --no-pager
else
    echo "Service not started. You can start it later with:"
    echo "  sudo systemctl start ${SERVICE_NAME}.service"
fi

echo ""
echo "Setup completed successfully!"
echo ""
echo "Configuration Summary:"
echo "  Robot ID: $ROBOT_ID"
echo "  User: $USER_NAME"
echo "  Service: ${SERVICE_NAME}.service"
echo ""
echo "Key Settings Applied:"
echo "  GPS: baudrate=$GPS_BAUDRATE, timeout=$GPS_TIMEOUT, update_rate=${GPS_UPDATE_RATE}Hz"
echo "  IMU: heading_offset=$IMU_HEADING_OFFSET" 
echo "  Locomotion: max_vel=$LOCOMOTION_MAX_VEL"
echo "  Monitoring: sensor_timeout=$MONITOR_SENSOR_TIMEOUT"
echo ""
echo "Parameter files generated in: $WORK_DIR/pioneer_ws/config/nodes/"
echo ""
echo "Useful commands:"
echo "  Check service: sudo systemctl status ${SERVICE_NAME}.service"
echo "  View logs: journalctl -u ${SERVICE_NAME}.service -f"
echo "  Rebuild: cd $WORK_DIR && colcon build --symlink-install"
echo "  Regenerate params: cd $WORK_DIR/pioneer_ws/scripts && ./setup.sh --regen-params-only"
echo ""
echo "To change configuration:"
echo "  1. Edit: $CONFIG_FILE"
echo "  2. Run: ./setup.sh --regen-params-only"
echo "  3. Restart: sudo systemctl restart ${SERVICE_NAME}.service"