import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion, Pose2D
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Int16, Bool

import math
import threading
import time

from pioneer_interfaces.srv import RefGPS

import datetime
import os
from math import sin, cos, asin, atan2, sqrt, degrees, pi, radians


class PoseConverter(Node):
    def __init__(self, n_rover=6):
        self.robot_id = os.getenv("ROBOT_ID")
        super().__init__(f'{self.robot_id}_pose_converter')
        
        # Initialize all data variables
        self.ref_lat = None
        self.ref_lon = None
        self.lat = None
        self.lon = None
        self.gps_status = None
        self.quaternion = None
        self.euler_x = None
        self.calibration = None
        self.lat_offset = 0.0
        self.lon_offset = 0.0
        
        # Flags for data availability
        self.gps_data_available = False
        self.imu_data_available = False
        self.last_gps_time = time.time()
        self.last_imu_time = time.time()
        
        # Reset command acknowledgment
        self.reset_gps_pending = False
        self.reset_imu_pending = False
        self.reset_timeout = 5.0  # seconds
        self.reset_gps_timer = None
        self.reset_imu_timer = None
        
        # Thread safety lock
        self.data_lock = threading.Lock()

        # Create subscriptions with error handling
        self.create_robust_subscriptions()
        
        # Create publishers
        self.pose_publisher = self.create_publisher(
            Pose2D,
            f'/{self.robot_id}/pose2D',
            5)
        
        # Service client for reference GPS
        self.cli = self.create_client(RefGPS, 'reference_gps')
        
        # Timer for main processing loop
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Timer for data health monitoring
        health_check_period = 1.0  # seconds
        self.health_timer = self.create_timer(health_check_period, self.health_check_callback)
        
        # Initialize reference GPS
        self.initialize_reference_gps()

    def create_robust_subscriptions(self):
        """Create subscriptions with error handling"""
        try:
            self.gps_subscription = self.create_subscription(
                NavSatFix,
                f"/{self.robot_id}/gps1",
                self.gps_callback,
                1)
        except Exception as e:
            self.get_logger().error(f"Failed to create GPS subscription: {e}")
            
        try:
            self.quaternion_subscription = self.create_subscription(
                Quaternion,
                f"/{self.robot_id}/imu/quaternion",
                self.quaternion_callback,
                1)
        except Exception as e:
            self.get_logger().error(f"Failed to create quaternion subscription: {e}")
            
        try:
            self.euler_subscription = self.create_subscription(
                Float32MultiArray,
                f"/{self.robot_id}/imu/eulerAngle",
                self.euler_callback,
                1)
        except Exception as e:
            self.get_logger().error(f"Failed to create euler subscription: {e}")
            
        try:
            self.reset_gps_subscription = self.create_subscription(
                Bool,
                f"/{self.robot_id}/reset_gps",
                self.reset_gps_callback,
                1)
        except Exception as e:
            self.get_logger().error(f"Failed to create reset GPS subscription: {e}")
            
        try:
            self.reset_imu_subscription = self.create_subscription(
                Bool,
                f"/{self.robot_id}/reset_imu",
                self.reset_imu_callback,
                1)
        except Exception as e:
            self.get_logger().error(f"Failed to create reset IMU subscription: {e}")

    def initialize_reference_gps(self):
        """Initialize reference GPS with retry mechanism"""
        def wait_for_service():
            retry_count = 0
            max_retries = 10
            while not self.cli.wait_for_service(timeout_sec=2.0) and retry_count < max_retries:
                self.get_logger().warn(f'Reference GPS service not available, waiting... (attempt {retry_count + 1})')
                retry_count += 1
                time.sleep(1.0)
            
            if retry_count < max_retries:
                self.request_reference_gps()
            else:
                self.get_logger().error('Reference GPS service failed to become available')
        
        # Run service waiting in a separate thread to avoid blocking
        threading.Thread(target=wait_for_service, daemon=True).start()
    
    def request_reference_gps(self):
        """Request reference GPS with improved error handling"""
        try:
            self.req = RefGPS.Request()
            self.req.robot_id = self.robot_id
            self.future = self.cli.call_async(self.req)
            self.future.add_done_callback(self.srv_callback)
            self.get_logger().info(f'[{self.req.robot_id}]Sent request for reference GPS')
        except Exception as e:
            self.get_logger().error(f'Failed to request reference GPS: {e}')
    
    def reset_gps_callback(self, msg):
        """Handle GPS reset with acknowledgment"""
        with self.data_lock:
            if self.lat is not None and self.ref_lat is not None:
                self.lat_offset = self.lat - self.ref_lat
                self.lon_offset = self.lon - self.ref_lon
                self.get_logger().info(f'[{self.robot_id}]GPS reset completed: offsets {self.lat_offset:.6f}, {self.lon_offset:.6f}')
                self.reset_gps_pending = False
                if self.reset_gps_timer:
                    self.reset_gps_timer.cancel()
            else:
                self.get_logger().warn(f'[{self.robot_id}]GPS reset failed: missing data')
                self.schedule_reset_retry('gps')

    def reset_imu_callback(self, msg):
        """Handle IMU reset with acknowledgment"""
        with self.data_lock:
            if self.euler_x is not None:
                self.calibration = self.euler_x
                self.get_logger().info(f'[{self.robot_id}]IMU reset completed: calibration {self.calibration:.2f}')
                self.reset_imu_pending = False
                if self.reset_imu_timer:
                    self.reset_imu_timer.cancel()
            else:
                self.get_logger().warn(f'[{self.robot_id}]IMU reset failed: missing data')
                self.schedule_reset_retry('imu')

    def schedule_reset_retry(self, sensor_type):
        """Schedule retry for reset commands"""
        def retry_reset():
            time.sleep(2.0)  # Wait before retry
            if sensor_type == 'gps' and self.reset_gps_pending:
                self.get_logger().info(f'[{self.robot_id}]Retrying GPS reset')
                self.reset_gps_callback(Bool())
            elif sensor_type == 'imu' and self.reset_imu_pending:
                self.get_logger().info(f'[{self.robot_id}]Retrying IMU reset')
                self.reset_imu_callback(Bool())
        
        threading.Thread(target=retry_reset, daemon=True).start()
    
    def srv_callback(self, future):
        """Handle reference GPS service response"""
        try:
            result = future.result()
            with self.data_lock:
                self.ref_lat = result.gps.latitude
                self.ref_lon = result.gps.longitude
            self.get_logger().info(f'Reference GPS received: {self.ref_lat:.6f}, {self.ref_lon:.6f}')
        except Exception as e:
            self.get_logger().error(f'Failed to get reference GPS: {e}')
            # Retry after delay
            threading.Timer(5.0, self.request_reference_gps).start()
    
    def gps_callback(self, msg):
        """Handle GPS data with validation"""
        try:
            with self.data_lock:
                # Validate GPS data
                if msg.latitude != 0.0 and msg.longitude != 0.0:
                    self.lat = msg.latitude
                    self.lon = msg.longitude
                    self.gps_status = msg.status.status
                    self.gps_data_available = True
                    self.last_gps_time = time.time()
                else:
                    self.get_logger().debug('Invalid GPS data received')
        except Exception as e:
            self.get_logger().error(f'GPS callback error: {e}')
    
    def quaternion_callback(self, msg):
        """Handle quaternion data with validation"""
        try:
            with self.data_lock:
                self.quaternion = msg
                self.last_imu_time = time.time()
        except Exception as e:
            self.get_logger().error(f'Quaternion callback error: {e}')
    
    def euler_callback(self, msg):
        """Handle Euler angle data with validation"""
        try:
            if len(msg.data) >= 3:
                with self.data_lock:
                    self.euler_x = msg.data[0]
                    self.euler_y = msg.data[1]
                    self.euler_z = msg.data[2]
                    if self.calibration is None:
                        self.calibration = self.euler_x
                    self.imu_data_available = True
                    self.last_imu_time = time.time()
        except Exception as e:
            self.get_logger().error(f'Euler callback error: {e}')

    def health_check_callback(self):
        """Monitor sensor health and data availability"""
        current_time = time.time()
        
        # Check GPS health
        if current_time - self.last_gps_time > 5.0:  # 5 seconds timeout
            self.gps_data_available = False
            self.get_logger().warn('GPS data timeout detected')
        
        # Check IMU health
        if current_time - self.last_imu_time > 5.0:  # 5 seconds timeout
            self.imu_data_available = False
            self.get_logger().warn('IMU data timeout detected')
        
        # Check reference GPS
        if self.ref_lat is None or self.ref_lon is None:
            self.get_logger().warn('Reference GPS not available, retrying...')
            self.initialize_reference_gps()
    
    def timer_callback(self):
        """Main processing loop with comprehensive error handling"""
        try:
            with self.data_lock:
                # Check if all required data is available
                if not self.validate_data():
                    return
                
                # Create and publish pose message
                msg = Pose2D()
                msg.x, msg.y = self.convert_gps_to_pose(
                    self.lat - self.lat_offset, 
                    self.lon - self.lon_offset, 
                    self.ref_lat, 
                    self.ref_lon
                )
                msg.theta = self.degree_to_radian_pi_range(self.euler_x - self.calibration)
                
                self.pose_publisher.publish(msg)
                self.get_logger().debug(f"Published pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")
                
        except Exception as e:
            self.get_logger().error(f'Timer callback error: {e}')

    def validate_data(self):
        """Validate that all required data is available"""
        if self.ref_lat is None or self.ref_lon is None:
            self.get_logger().debug("Reference GPS not set")
            return False
        
        if not self.gps_data_available or self.lat is None or self.lon is None:
            self.get_logger().debug("GPS data not available")
            return False
        
        if not self.imu_data_available or self.quaternion is None or self.euler_x is None or self.calibration is None:
            self.get_logger().debug("IMU data not available")
            return False
        
        return True
        
    def degree_to_radian_pi_range(self, degree):
        """Convert degree to radian with range normalization"""
        try:
            # Convert degree to radian
            rad = math.radians(degree)
            # Normalize to [-π, π)
            rad = (rad + math.pi) % (2 * math.pi) - math.pi
            return rad
        except Exception as e:
            self.get_logger().error(f'Degree to radian conversion error: {e}')
            return 0.0
    
    def convert_gps_to_pose(self, cur_lat, cur_lon, ref_lat, ref_lon):
        """Convert GPS coordinates to local pose with error handling"""
        try:
            R = 6371000  # Earth radius in meters
            
            delta_lat = radians(cur_lat - ref_lat)
            delta_lon = radians(cur_lon - ref_lon)
            
            ref_lat_rad = radians(ref_lat)
            
            x = R * delta_lon * cos(ref_lat_rad)
            y = R * delta_lat
            
            return x, y
        except Exception as e:
            self.get_logger().error(f'GPS to pose conversion error: {e}')
            return 0.0, 0.0

def main(args=None):
    rclpy.init(args=args)
    try:
        converter = PoseConverter()
        rclpy.spin(converter)
    except Exception as e:
        print(f"Converter failed: {e}")
    finally:
        try:
            converter.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()