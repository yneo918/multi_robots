#!/bin/bash

# Systemd-compatible ROS2 startup script
# This version is designed to work properly with systemd services

# Configuration
LOG_DIR="$HOME/ros2_logs"
LOG_FILE="$LOG_DIR/systemd_startup_$(date +%Y%m%d_%H%M%S).log"
PID_FILE="$HOME/.ros2_systemd.pid"
STATUS_FILE="$HOME/.ros2_status"

# Create log directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Logging function optimized for systemd
log_message() {
    local level="$1"
    local message="$2"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    
    # Log to file
    echo "[$timestamp] [$level] $message" >> "$LOG_FILE"
    
    # Also log to stdout for systemd journal
    echo "[$level] $message"
}

# Error handling function
handle_error() {
    local exit_code=$?
    local line_number=$1
    log_message "ERROR" "Script failed at line $line_number with exit code $exit_code"
    echo "FAILED" > "$STATUS_FILE"
    cleanup_and_exit $exit_code
}

# Cleanup function for systemd
cleanup_and_exit() {
    local exit_code=${1:-0}
    log_message "INFO" "Systemd cleanup initiated..."
    
    # Update status
    echo "STOPPING" > "$STATUS_FILE"
    
    # Kill ROS2 processes gracefully
    if [[ -f "$PID_FILE" ]]; then
        while read pid; do
            if kill -0 "$pid" 2>/dev/null; then
                log_message "INFO" "Terminating process $pid"
                kill -TERM "$pid" 2>/dev/null
            fi
        done < "$PID_FILE"
        
        # Wait for graceful shutdown
        sleep 5
        
        # Force kill if necessary
        while read pid; do
            if kill -0 "$pid" 2>/dev/null; then
                log_message "WARN" "Force killing process $pid"
                kill -KILL "$pid" 2>/dev/null
            fi
        done < "$PID_FILE"
        
        rm -f "$PID_FILE"
    fi
    
    # Additional cleanup
    pkill -f "ros2 launch" 2>/dev/null || true
    pkill -f "ros2 run" 2>/dev/null || true
    
    echo "STOPPED" > "$STATUS_FILE"
    log_message "INFO" "Cleanup complete"
    exit $exit_code
}

# Signal handlers for systemd
trap 'log_message "INFO" "Received SIGTERM"; cleanup_and_exit 0' TERM
trap 'log_message "INFO" "Received SIGINT"; cleanup_and_exit 0' INT
trap 'handle_error $LINENO' ERR

# Check prerequisites
check_prerequisites() {
    log_message "INFO" "Checking prerequisites..."
    
    # Check ROBOT_ID
    if [[ -z "$ROBOT_ID" ]]; then
        log_message "ERROR" "ROBOT_ID environment variable is not set"
        return 1
    fi
    
    # Check ROS2 installation
    if ! command -v ros2 &> /dev/null; then
        log_message "ERROR" "ROS2 not found in PATH"
        return 1
    fi
    
    # Check workspace
    if [[ ! -f "$HOME/ros2_ws/install/setup.bash" ]]; then
        log_message "ERROR" "ROS2 workspace not found or not built"
        return 1
    fi
    
    log_message "INFO" "Prerequisites check passed"
    return 0
}

# Setup environment for systemd
setup_environment() {
    log_message "INFO" "Setting up ROS2 environment..."
    
    # Ensure we're in the right directory
    cd "$HOME/ros2_ws" || {
        log_message "ERROR" "Cannot change to workspace directory"
        return 1
    }
    
    # Source ROS2 - handle potential sourcing issues
    if [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
        source /opt/ros/jazzy/setup.bash
    else
        log_message "ERROR" "ROS2 installation not found"
        return 1
    fi
    
    # Source workspace
    source ~/ros2_ws/install/setup.bash || {
        log_message "ERROR" "Failed to source workspace"
        return 1
    }
    
    # Set environment variables for systemd
    export ROS_LOG_DIR="$LOG_DIR"
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    export RCUTILS_LOGGING_SEVERITY_THRESHOLD=INFO
    export RCUTILS_LOGGING_USE_STDOUT=1
    
    # Disable interactive features
    export ROS_PYTHON_LOG_CONFIG_PATH=""
    
    log_message "INFO" "Environment setup completed"
    return 0
}

# Wait for devices with timeout
wait_for_devices() {
    log_message "INFO" "Waiting for devices..."
    
    local max_wait=60
    local counter=0
    
    # Wait for at least one serial device (GPS)
    while [[ $counter -lt $max_wait ]]; do
        if [[ -e "/dev/ttyUSB0" ]] || [[ -e "/dev/ttyACM0" ]]; then
            log_message "INFO" "Serial device found"
            break
        fi
        sleep 1
        counter=$((counter + 1))
    done
    
    if [[ $counter -ge $max_wait ]]; then
        log_message "WARN" "No serial devices found after ${max_wait}s"
    fi
    
    # Check I2C for IMU
    if [[ -e "/dev/i2c-1" ]]; then
        log_message "INFO" "I2C device available"
    else
        log_message "WARN" "I2C device not found"
    fi
    
    # Give devices time to stabilize
    sleep 3
}

# Start ROS2 processes for systemd
start_ros_processes() {
    log_message "INFO" "Starting ROS2 processes..."
    
    # Initialize PID tracking
    echo "" > "$PID_FILE"
    
    # Start rover launch system
    log_message "INFO" "Starting rover launch system"
    ros2 launch rover_launch rover.launch.py > "$LOG_DIR/rover_launch.log" 2>&1 &
    local launch_pid=$!
    echo "$launch_pid" >> "$PID_FILE"
    
    # Check if launch started successfully
    sleep 5
    if ! kill -0 "$launch_pid" 2>/dev/null; then
        log_message "ERROR" "Rover launch failed to start"
        return 1
    fi
    
    log_message "INFO" "Rover launch started (PID: $launch_pid)"
    
    # Wait for launch system to initialize
    sleep 10
    
    # Start pose converter
    log_message "INFO" "Starting pose converter"
    ros2 run convert_pose converter > "$LOG_DIR/pose_converter.log" 2>&1 &
    local converter_pid=$!
    echo "$converter_pid" >> "$PID_FILE"
    
    # Check if converter started successfully
    sleep 3
    if ! kill -0 "$converter_pid" 2>/dev/null; then
        log_message "ERROR" "Pose converter failed to start"
        return 1
    fi
    
    log_message "INFO" "Pose converter started (PID: $converter_pid)"
    return 0
}

# Verify system startup
verify_startup() {
    log_message "INFO" "Verifying system startup..."
    
    local max_verification_time=60
    local check_interval=5
    local elapsed=0
    
    # Expected nodes
    local expected_nodes=(
        "${ROBOT_ID}_pose_converter"
        "${ROBOT_ID}_gps1"
        "${ROBOT_ID}_imu"
    )
    
    while [[ $elapsed -lt $max_verification_time ]]; do
        local nodes_found=0
        local total_nodes=${#expected_nodes[@]}
        
        # Check if we can get node list
        if ros2_nodes=$(ros2 node list 2>/dev/null); then
            for node in "${expected_nodes[@]}"; do
                if echo "$ros2_nodes" | grep -q "$node"; then
                    nodes_found=$((nodes_found + 1))
                fi
            done
            
            log_message "INFO" "Startup verification: $nodes_found/$total_nodes nodes running"
            
            # Consider successful if at least critical nodes are running
            if [[ $nodes_found -ge 2 ]]; then
                log_message "INFO" "System startup verification successful"
                return 0
            fi
        else
            log_message "WARN" "Cannot query ROS2 node list"
        fi
        
        sleep $check_interval
        elapsed=$((elapsed + check_interval))
    done
    
    log_message "ERROR" "System startup verification failed"
    return 1
}

# Main execution for systemd
main() {
    # Update status
    echo "STARTING" > "$STATUS_FILE"
    
    log_message "INFO" "=== ROS2 Systemd Startup Started ==="
    log_message "INFO" "Robot ID: $ROBOT_ID"
    log_message "INFO" "Service PID: $$"
    
    # Startup sequence
    if ! check_prerequisites; then
        echo "FAILED" > "$STATUS_FILE"
        exit 1
    fi
    
    if ! setup_environment; then
        echo "FAILED" > "$STATUS_FILE"
        exit 1
    fi
    
    wait_for_devices
    
    if ! start_ros_processes; then
        echo "FAILED" > "$STATUS_FILE"
        exit 1
    fi
    
    if ! verify_startup; then
        echo "FAILED" > "$STATUS_FILE"
        exit 1
    fi
    
    # Update status to running
    echo "RUNNING" > "$STATUS_FILE"
    log_message "INFO" "=== ROS2 System Started Successfully ==="
    
    # Keep the script running for systemd
    # This replaces the monitoring loop from the original script
    while true; do
        # Check if our main processes are still alive
        local processes_alive=0
        if [[ -f "$PID_FILE" ]]; then
            while read pid; do
                if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
                    processes_alive=$((processes_alive + 1))
                fi
            done < "$PID_FILE"
        fi
        
        if [[ $processes_alive -eq 0 ]]; then
            log_message "ERROR" "All ROS2 processes have died"
            echo "FAILED" > "$STATUS_FILE"
            exit 1
        fi
        
        # Sleep and check again
        sleep 30
    done
}

# Execute main function
main "$@"