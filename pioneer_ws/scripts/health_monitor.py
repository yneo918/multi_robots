#!/usr/bin/env python3

"""
Complete Health monitoring script for Pioneer ROS system
This script monitors the health of ROS nodes and can restart them if needed
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion, Pose2D
from std_msgs.msg import Float32MultiArray, Bool
import subprocess
import time
import threading
import sys
import os
import yaml
import signal
import psutil
import json
from datetime import datetime, timedelta


class HealthMonitor(Node):
    def __init__(self, robot_id, config_file=None):
        super().__init__(f'{robot_id}_health_monitor')
        self.robot_id = robot_id
        
        # Load configuration
        self.load_config(config_file)
        
        # Health tracking
        self.last_gps_time = 0
        self.last_imu_quaternion_time = 0
        self.last_imu_euler_time = 0
        self.last_pose_time = 0
        
        # Data quality tracking
        self.gps_data_count = 0
        self.imu_data_count = 0
        self.pose_data_count = 0
        self.invalid_gps_count = 0
        self.invalid_imu_count = 0
        
        # Node status tracking
        self.node_status = {
            'gps': {
                'healthy': False, 
                'last_restart': 0, 
                'restart_count': 0,
                'process_pid': None,
                'data_rate': 0.0,
                'error_count': 0
            },
            'imu': {
                'healthy': False, 
                'last_restart': 0, 
                'restart_count': 0,
                'process_pid': None,
                'data_rate': 0.0,
                'error_count': 0
            },
            'converter': {
                'healthy': False, 
                'last_restart': 0, 
                'restart_count': 0,
                'process_pid': None,
                'data_rate': 0.0,
                'error_count': 0
            }
        }
        
        # System status
        self.system_start_time = time.time()
        self.total_restarts = 0
        self.last_status_log = 0
        
        # Create subscriptions to monitor data
        self.create_health_subscriptions()
        
        # Create service for external health queries
        self.create_health_publishers()
        
        # Start monitoring timers
        self.monitor_timer = self.create_timer(
            self.config.get('monitoring_interval', 5.0),
            self.monitor_callback
        )
        
        self.statistics_timer = self.create_timer(
            self.config.get('statistics_interval', 30.0),
            self.statistics_callback
        )
        
        # Shutdown flag
        self.shutdown_requested = False
        
        # Lock for thread safety
        self.status_lock = threading.Lock()
        
        self.get_logger().info(f'Health monitor started for {robot_id}')
        self.get_logger().info(f'Monitoring interval: {self.config.get("monitoring_interval", 5.0)}s')
        self.get_logger().info(f'Sensor timeout: {self.config.get("sensor_timeout", 10.0)}s')

    def load_config(self, config_file):
        """Load configuration from YAML file"""
        # Default configuration
        self.config = {
            'sensor_timeout': 10.0,
            'max_restarts_per_hour': 3,
            'restart_cooldown': 300.0,  # 5 minutes
            'monitoring_interval': 5.0,
            'statistics_interval': 30.0,
            'log_interval': 60.0,
            'data_rate_threshold': 0.5,  # minimum data rate (Hz)
            'node_restart_commands': {
                'gps': f'ros2 run gps_core gps_revised --ros-args -p robot_id:={self.robot_id if hasattr(self, "robot_id") else "pX"}',
                'imu': f'ros2 run imu_core run_imu --ros-args -p robot_id:={self.robot_id if hasattr(self, "robot_id") else "pX"}',
                'converter': f'ros2 run convert_pose converter --ros-args -p robot_id:={self.robot_id if hasattr(self, "robot_id") else "pX"}'
            }
        }
        
        if config_file and os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    file_config = yaml.safe_load(f)
                
                # Update config with file values
                if 'monitoring' in file_config:
                    self.config.update(file_config['monitoring'])
                    
                self.get_logger().info(f'Loaded configuration from {config_file}')
                    
            except Exception as e:
                self.get_logger().warn(f'Failed to load config file {config_file}: {e}')

    def create_health_subscriptions(self):
        """Create subscriptions to monitor sensor data"""
        try:
            # GPS monitoring
            self.gps_sub = self.create_subscription(
                NavSatFix,
                f'/{self.robot_id}/gps1',
                self.gps_health_callback,
                1
            )
            
            # IMU monitoring
            self.imu_quat_sub = self.create_subscription(
                Quaternion,
                f'/{self.robot_id}/imu/quaternion',
                self.imu_quaternion_health_callback,
                1
            )
            
            self.imu_euler_sub = self.create_subscription(
                Float32MultiArray,
                f'/{self.robot_id}/imu/eulerAngle',
                self.imu_euler_health_callback,
                1
            )
            
            # Pose converter monitoring
            self.pose_sub = self.create_subscription(
                Pose2D,
                f'/{self.robot_id}/pose2D',
                self.pose_health_callback,
                1
            )
            
            self.get_logger().info('Health monitoring subscriptions created')
            
        except Exception as e:
            self.get_logger().error(f'Failed to create health subscriptions: {e}')

    def create_health_publishers(self):
        """Create publishers for health status"""
        try:
            # Health status publisher
            self.health_status_pub = self.create_publisher(
                Float32MultiArray,
                f'/{self.robot_id}/health_status',
                1
            )
            
            # Reset command publishers for emergency resets
            self.gps_reset_pub = self.create_publisher(
                Bool,
                f'/{self.robot_id}/reset_gps',
                1
            )
            
            self.imu_reset_pub = self.create_publisher(
                Bool,
                f'/{self.robot_id}/reset_imu',
                1
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to create health publishers: {e}')

    def gps_health_callback(self, msg):
        """Monitor GPS health"""
        current_time = time.time()
        
        with self.status_lock:
            self.last_gps_time = current_time
            self.gps_data_count += 1
            
            # Check data validity
            if msg.latitude != 0.0 and msg.longitude != 0.0 and msg.status.status > 0:
                self.node_status['gps']['healthy'] = True
                self.node_status['gps']['error_count'] = max(0, self.node_status['gps']['error_count'] - 1)
            else:
                self.invalid_gps_count += 1
                self.node_status['gps']['error_count'] += 1
                
                if self.invalid_gps_count > 10:  # Too many invalid readings
                    self.get_logger().warn('GPS providing invalid data consistently')

    def imu_quaternion_health_callback(self, msg):
        """Monitor IMU quaternion health"""
        current_time = time.time()
        
        with self.status_lock:
            self.last_imu_quaternion_time = current_time
            
            # Check data validity
            if all(q is not None and abs(q) < 10.0 for q in [msg.x, msg.y, msg.z, msg.w]):
                self.node_status['imu']['error_count'] = max(0, self.node_status['imu']['error_count'] - 1)
            else:
                self.invalid_imu_count += 1
                self.node_status['imu']['error_count'] += 1

    def imu_euler_health_callback(self, msg):
        """Monitor IMU Euler health"""
        current_time = time.time()
        
        with self.status_lock:
            self.last_imu_euler_time = current_time
            self.imu_data_count += 1
            
            # IMU is healthy if both quaternion and euler data are recent
            if (current_time - self.last_imu_quaternion_time < self.config['sensor_timeout'] and
                current_time - self.last_imu_euler_time < self.config['sensor_timeout']):
                self.node_status['imu']['healthy'] = True

    def pose_health_callback(self, msg):
        """Monitor pose converter health"""
        current_time = time.time()
        
        with self.status_lock:
            self.last_pose_time = current_time
            self.pose_data_count += 1
            self.node_status['converter']['healthy'] = True

    def monitor_callback(self):
        """Main monitoring callback - check health and restart nodes if needed"""
        if self.shutdown_requested:
            return
            
        current_time = time.time()
        
        with self.status_lock:
            # Check GPS health
            if current_time - self.last_gps_time > self.config['sensor_timeout']:
                if self.node_status['gps']['healthy']:
                    self.get_logger().warn('GPS data timeout detected')
                self.node_status['gps']['healthy'] = False
                self.handle_unhealthy_node('gps')
            
            # Check IMU health
            if (current_time - self.last_imu_quaternion_time > self.config['sensor_timeout'] or
                current_time - self.last_imu_euler_time > self.config['sensor_timeout']):
                if self.node_status['imu']['healthy']:
                    self.get_logger().warn('IMU data timeout detected')
                self.node_status['imu']['healthy'] = False
                self.handle_unhealthy_node('imu')
            
            # Check converter health
            if current_time - self.last_pose_time > self.config['sensor_timeout']:
                if self.node_status['converter']['healthy']:
                    self.get_logger().warn('Converter data timeout detected')
                self.node_status['converter']['healthy'] = False
                self.handle_unhealthy_node('converter')
            
            # Update data rates
            self.update_data_rates(current_time)
            
            # Check for excessive error counts
            self.check_error_counts()
        
        # Publish health status
        self.publish_health_status()
        
        # Log status periodically
        if current_time - self.last_status_log > self.config.get('log_interval', 60.0):
            self.log_health_status()
            self.last_status_log = current_time

    def update_data_rates(self, current_time):
        """Update data rate calculations"""
        time_window = 30.0  # 30 second window
        
        if hasattr(self, 'last_rate_update'):
            dt = current_time - self.last_rate_update
            if dt > 0:
                self.node_status['gps']['data_rate'] = self.gps_data_count / dt
                self.node_status['imu']['data_rate'] = self.imu_data_count / dt
                self.node_status['converter']['data_rate'] = self.pose_data_count / dt
                
                # Reset counters
                self.gps_data_count = 0
                self.imu_data_count = 0
                self.pose_data_count = 0
        
        self.last_rate_update = current_time

    def check_error_counts(self):
        """Check for excessive error counts and trigger resets if needed"""
        for node_name, status in self.node_status.items():
            if status['error_count'] > 20:  # Too many errors
                self.get_logger().warn(f'{node_name} has excessive errors ({status["error_count"]}), triggering reset')
                if node_name == 'gps':
                    self.send_reset_command('gps')
                elif node_name == 'imu':
                    self.send_reset_command('imu')
                status['error_count'] = 0  # Reset counter

    def send_reset_command(self, sensor_type):
        """Send reset command to sensor"""
        try:
            msg = Bool()
            msg.data = True
            
            if sensor_type == 'gps':
                self.gps_reset_pub.publish(msg)
                self.get_logger().info('Sent GPS reset command')
            elif sensor_type == 'imu':
                self.imu_reset_pub.publish(msg)
                self.get_logger().info('Sent IMU reset command')
                
        except Exception as e:
            self.get_logger().error(f'Failed to send {sensor_type} reset command: {e}')

    def handle_unhealthy_node(self, node_name):
        """Handle an unhealthy node by attempting restart"""
        current_time = time.time()
        node_info = self.node_status[node_name]
        
        # Check if we should restart the node
        if not self.should_restart_node(node_name, current_time):
            return
        
        self.get_logger().warn(f'Node {node_name} is unhealthy, attempting restart')
        
        # Update restart tracking
        node_info['last_restart'] = current_time
        node_info['restart_count'] += 1
        self.total_restarts += 1
        
        # Restart the node in a separate thread
        threading.Thread(
            target=self.restart_node,
            args=(node_name,),
            daemon=True
        ).start()

    def should_restart_node(self, node_name, current_time):
        """Determine if a node should be restarted"""
        node_info = self.node_status[node_name]
        
        # Check cooldown period
        if current_time - node_info['last_restart'] < self.config['restart_cooldown']:
            return False
        
        # Check restart count in the last hour
        one_hour_ago = current_time - 3600
        if node_info['last_restart'] > one_hour_ago:
            if node_info['restart_count'] >= self.config['max_restarts_per_hour']:
                self.get_logger().error(
                    f'Node {node_name} has been restarted too many times recently '
                    f'({node_info["restart_count"]} times), manual intervention required'
                )
                return False
        else:
            # Reset restart count if more than an hour has passed
            node_info['restart_count'] = 0
        
        return True

    def restart_node(self, node_name):
        """Restart a specific node"""
        try:
            self.get_logger().info(f'Restarting {node_name} node...')
            
            # Get restart command
            restart_cmd = self.config['node_restart_commands'].get(node_name)
            if not restart_cmd:
                self.get_logger().error(f'No restart command configured for {node_name}')
                return
            
            # Kill existing process first
            self.kill_node_process(node_name)
            
            # Wait a moment before restarting
            time.sleep(3.0)
            
            # Start the new process
            env = os.environ.copy()
            env['ROBOT_ID'] = self.robot_id
            
            # Update command with actual robot ID
            restart_cmd = restart_cmd.replace('pX', self.robot_id)
            
            process = subprocess.Popen(
                restart_cmd.split(),
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid  # Create new process group
            )
            
            # Store process info
            with self.status_lock:
                self.node_status[node_name]['process_pid'] = process.pid
            
            self.get_logger().info(f'Node {node_name} restarted with PID {process.pid}')
            
            # Give the node time to initialize
            time.sleep(2.0)
            
        except Exception as e:
            self.get_logger().error(f'Failed to restart {node_name}: {e}')

    def kill_node_process(self, node_name):
        """Kill existing node process"""
        try:
            # Use pkill to kill processes by name pattern
            node_patterns = {
                'gps': 'gps_revised',
                'imu': 'run_imu', 
                'converter': 'converter'
            }
            
            pattern = node_patterns.get(node_name)
            if pattern:
                # First try graceful termination
                subprocess.run(['pkill', '-TERM', '-f', pattern], check=False)
                time.sleep(2.0)
                
                # Then force kill if still running
                subprocess.run(['pkill', '-KILL', '-f', pattern], check=False)
                time.sleep(1.0)
                
                self.get_logger().info(f'Killed existing {node_name} processes')
                
        except Exception as e:
            self.get_logger().warn(f'Error killing {node_name} process: {e}')

    def publish_health_status(self):
        """Publish current health status"""
        try:
            msg = Float32MultiArray()
            
            # Pack status data: [gps_healthy, imu_healthy, converter_healthy, 
            #                   gps_rate, imu_rate, converter_rate, total_restarts]
            msg.data = [
                1.0 if self.node_status['gps']['healthy'] else 0.0,
                1.0 if self.node_status['imu']['healthy'] else 0.0,
                1.0 if self.node_status['converter']['healthy'] else 0.0,
                float(self.node_status['gps']['data_rate']),
                float(self.node_status['imu']['data_rate']),
                float(self.node_status['converter']['data_rate']),
                float(self.total_restarts)
            ]
            
            self.health_status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish health status: {e}')

    def statistics_callback(self):
        """Generate and log statistics"""
        current_time = time.time()
        uptime = current_time - self.system_start_time
        
        stats = {
            'uptime_hours': uptime / 3600,
            'total_restarts': self.total_restarts,
            'node_status': {}
        }
        
        with self.status_lock:
            for node_name, status in self.node_status.items():
                stats['node_status'][node_name] = {
                    'healthy': status['healthy'],
                    'restart_count': status['restart_count'],
                    'data_rate': status['data_rate'],
                    'error_count': status['error_count']
                }
        
        # Log detailed statistics
        self.get_logger().info(
            f"System Statistics - Uptime: {uptime/3600:.1f}h, "
            f"Total Restarts: {self.total_restarts}, "
            f"GPS: {'OK' if stats['node_status']['gps']['healthy'] else 'FAIL'} "
            f"({stats['node_status']['gps']['data_rate']:.1f}Hz), "
            f"IMU: {'OK' if stats['node_status']['imu']['healthy'] else 'FAIL'} "
            f"({stats['node_status']['imu']['data_rate']:.1f}Hz), "
            f"Converter: {'OK' if stats['node_status']['converter']['healthy'] else 'FAIL'} "
            f"({stats['node_status']['converter']['data_rate']:.1f}Hz)"
        )

    def log_health_status(self):
        """Log current health status"""
        with self.status_lock:
            status_msgs = []
            for node_name, info in self.node_status.items():
                status = "HEALTHY" if info['healthy'] else "UNHEALTHY"
                restarts = info['restart_count']
                rate = info['data_rate']
                errors = info['error_count']
                status_msgs.append(
                    f"{node_name}: {status} (restarts: {restarts}, rate: {rate:.1f}Hz, errors: {errors})"
                )
            
            self.get_logger().info(f"Health Status - {', '.join(status_msgs)}")

    def get_system_info(self):
        """Get system resource information"""
        try:
            cpu_percent = psutil.cpu_percent()
            memory = psutil.virtual_memory()
            disk = psutil.disk_usage('/')
            
            return {
                'cpu_percent': cpu_percent,
                'memory_percent': memory.percent,
                'disk_percent': disk.percent,
                'memory_available_gb': memory.available / (1024**3)
            }
        except:
            return {}

    def shutdown(self):
        """Graceful shutdown"""
        self.shutdown_requested = True
        self.get_logger().info('Health monitor shutting down...')
        
        # Log final statistics
        uptime = time.time() - self.system_start_time
        self.get_logger().info(
            f'Final Statistics - Uptime: {uptime/3600:.1f}h, '
            f'Total Restarts: {self.total_restarts}'
        )


def signal_handler(signum, frame, monitor):
    """Handle shutdown signals"""
    print(f"\nReceived signal {signum}, shutting down health monitor...")
    monitor.shutdown()
    sys.exit(0)


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Health monitor for Pioneer ROS system')
    parser.add_argument('--robot-id', type=str, default=None,
                       help='Robot ID (default: from ROBOT_ID environment variable)')
    parser.add_argument('--config', type=str, default=None,
                       help='Configuration file path')
    parser.add_argument('--daemon', action='store_true',
                       help='Run as daemon (background process)')
    parser.add_argument('--verbose', action='store_true',
                       help='Enable verbose logging')
    
    args = parser.parse_args()
    
    # Get robot ID
    robot_id = args.robot_id
    if not robot_id:
        robot_id = os.getenv('ROBOT_ID')
        if not robot_id:
            print("Error: Robot ID not specified and ROBOT_ID environment variable not set")
            print("Use --robot-id argument or set ROBOT_ID environment variable")
            sys.exit(1)
    
    print(f"Starting health monitor for {robot_id}...")
    if args.config:
        print(f"Using config file: {args.config}")
    if args.daemon:
        print("Running in daemon mode")
    else:
        print("Press Ctrl+C to stop")
    
    # Initialize ROS
    rclpy.init()
    
    try:
        # Create health monitor
        monitor = HealthMonitor(robot_id, args.config)
        
        # Set up signal handlers
        signal.signal(signal.SIGINT, lambda s, f: signal_handler(s, f, monitor))
        signal.signal(signal.SIGTERM, lambda s, f: signal_handler(s, f, monitor))
        
        # Set log level
        if args.verbose:
            monitor.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        print(f"Health monitor for {robot_id} is running...")
        
        # Spin the node
        rclpy.spin(monitor)
        
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"Health monitor error: {e}")
        sys.exit(1)
    finally:
        try:
            monitor.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()