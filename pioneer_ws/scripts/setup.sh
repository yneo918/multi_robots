#!/bin/bash

# Improved setup script for Pioneer ROS system
# This script configures the system using the central config file

set -e  # Exit on any error

CONFIG_FILE="pioneer_ws/config/system_config.yaml"
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

# Read configuration values
ROBOT_ID=$(read_yaml "$CONFIG_FILE" "robot.id")
USER_NAME=$(read_yaml "$CONFIG_FILE" "user.name")
HOME_DIR=$(read_yaml "$CONFIG_FILE" "user.home_dir")
WORK_DIR="$HOME_DIR/ros2_ws"

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

# Check if running as correct user
if [ "$USER" != "$USER_NAME" ]; then
    echo "Warning: Running as user '$USER' but config specifies '$USER_NAME'"
    echo "This may cause permission issues"
fi

# Create necessary directories
echo "Creating directories..."
mkdir -p "$HOME_DIR/imu_calib"
mkdir -p "$HOME_DIR/.ros"

# Export environment variables for current session
export ROBOT_ID="$ROBOT_ID"
export IMU_OFFSET=0

# Update .bashrc with environment variables
echo "Updating .bashrc..."
BASHRC="$HOME_DIR/.bashrc"

# Remove existing ROBOT_ID exports
sed -i '/export ROBOT_ID=/d' "$BASHRC" 2>/dev/null || true
sed -i '/export IMU_OFFSET=/d' "$BASHRC" 2>/dev/null || true

# Add new exports
echo "" >> "$BASHRC"
echo "# Pioneer ROS Environment Variables" >> "$BASHRC"
echo "export ROBOT_ID=$ROBOT_ID" >> "$BASHRC"
echo "export IMU_OFFSET=0" >> "$BASHRC"

# Generate startup script
STARTUP_SCRIPT="$WORK_DIR/pioneer_ws/scripts/start_ros2.sh"
echo "Generating startup script: $STARTUP_SCRIPT"

cat > "$STARTUP_SCRIPT" << EOF
#!/bin/bash

# Auto-generated startup script for Pioneer ROS system
# Robot ID: $ROBOT_ID
# User: $USER_NAME

# Source ROS environment
source /opt/ros/jazzy/setup.bash
source $WORK_DIR/install/setup.bash

# Set environment variables
export ROBOT_ID=$ROBOT_ID
export IMU_OFFSET=0

# Change to workspace directory
cd $WORK_DIR

# Function to start node with retry logic
start_node_with_retry() {
    local package=\$1
    local executable=\$2
    local max_retries=3
    local retry_delay=5
    
    for ((i=1; i<=max_retries; i++)); do
        echo "Starting \$package/\$executable (attempt \$i/\$max_retries)"
        
        if ros2 run \$package \$executable; then
            echo "\$package/\$executable started successfully"
            return 0
        else
            echo "\$package/\$executable failed to start"
            if [ \$i -lt \$max_retries ]; then
                echo "Retrying in \$retry_delay seconds..."
                sleep \$retry_delay
            fi
        fi
    done
    
    echo "Failed to start \$package/\$executable after \$max_retries attempts"
    return 1
}

# Start launch file with error handling
echo "Starting Pioneer ROS system..."
echo "Robot ID: \$ROBOT_ID"
echo "Timestamp: \$(date)"

# Start the main launch file
ros2 launch rover_launch rover.launch.py &
LAUNCH_PID=\$!

# Start converter with retry logic
sleep 2  # Allow launch file to initialize
start_node_with_retry convert_pose converter &
CONVERTER_PID=\$!

# Wait for launch process
wait \$LAUNCH_PID
EOF

chmod +x "$STARTUP_SCRIPT"

# Generate systemd service file
SERVICE_NAME=$(read_yaml "$CONFIG_FILE" "service.name")
SERVICE_DESC=$(read_yaml "$CONFIG_FILE" "service.description")
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

echo "Generating systemd service file: $SERVICE_FILE"

sudo tee "$SERVICE_FILE" > /dev/null << EOF
[Unit]
Description=$SERVICE_DESC for $ROBOT_ID
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
Environment="ROBOT_ID=$ROBOT_ID"
Environment="IMU_OFFSET=0"
Environment="DISPLAY=:0"
User=$USER_NAME
Group=$USER_NAME
WorkingDirectory=$WORK_DIR
ExecStart=/bin/bash $STARTUP_SCRIPT
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal
KillMode=mixed
TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
EOF

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
echo "Configuration:"
echo "  Robot ID: $ROBOT_ID"
echo "  User: $USER_NAME"
echo "  Service: ${SERVICE_NAME}.service"
echo ""
echo "Useful commands:"
echo "  Check service status: sudo systemctl status ${SERVICE_NAME}.service"
echo "  Stop service: sudo systemctl stop ${SERVICE_NAME}.service"
echo "  Start service: sudo systemctl start ${SERVICE_NAME}.service"
echo "  View logs: journalctl -u ${SERVICE_NAME}.service -f"
echo "  Rebuild workspace: cd $WORK_DIR && colcon build --symlink-install"
echo ""
echo "To configure a different robot, edit: $CONFIG_FILE"
echo "Then run this script again."