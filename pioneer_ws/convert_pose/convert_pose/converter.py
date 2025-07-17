import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
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
        # Get robot ID from parameter or environment
        robot_id = os.getenv("ROBOT_ID", "pX")
        super().__init__(f'{robot_id}_pose_converter')
        
        # Declare parameters with defaults from system_config.yaml structure
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', robot_id),
                ('timer_period', 0.1),
                ('health_check_period', 1.0),
                ('reset_timeout', 5.0)
            ]
        )
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.health_check_period = self.get_parameter('health_check_period').get_parameter_value().double_value
        self.reset_timeout = self.get_parameter('reset_timeout').get_parameter_value().double_value
        
        # Log loaded configuration
        self.get_logger().info(f'Pose Converter Configuration:')
        self.get_logger().debug(f'  Robot ID: {self.robot_id}')
        self.get_logger().debug(f'  Timer Period: {self.timer_period}s')
        self.get_logger().debug(f'  Health Check Period: {self.health_check_period}s')
        self.get_logger().debug(f'  Reset Timeout: {self.reset_timeout}s')
        
        # Initialize all data variables
        self.ref_lat = None
        self.ref_lon = None
        self.lat = None
        self.lon = None
        self.gps_status = None
        self.quaternion = None
        self.euler_x = None
        self.euler_y = None
        self.euler_z = None
        self.calibration = None
        self.lat_offset = 0.0
        self.lon_offset = 0.0
        
        # Flags for data availability
        self.gps_data_available = False
        self.imu_data_available = False
        self.last_gps_time = time.time()
        self.last_imu_time = time.time()
        
        # IMU-only pose estimation variables
        self.imu_pose_x = 0.0
        self.imu_pose_y = 0.0
        self.imu_velocity_x = 0.0
        self.imu_velocity_y = 0.0
        self.imu_theta = 0.0
        self.imu_calibration = None
        self.last_imu_data_time = time.time()
        self.imu_data = None
        
        # Drift compensation variables
        self.accel_offset_x = 0.0
        self.accel_offset_y = 0.0
        self.velocity_decay_factor = 0.995  # Velocity decay to prevent drift
        self.accel_threshold = 0.05  # m/s² - minimum acceleration threshold
        self.calibration_samples = []
        self.calibration_sample_count = 100  # Number of samples for offset calibration
        self.is_calibrating = True
        
        # Reset command acknowledgment
        self.reset_gps_pending = False
        self.reset_imu_pending = False
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
        
        # Create publisher for IMU-only pose
        self.imu_pose_publisher = self.create_publisher(
            Pose2D,
            f'/{self.robot_id}/imu/pose2D',
            5)
        
        # Service client for reference GPS
        self.cli = self.create_client(RefGPS, 'reference_gps')
        
        # Timer for main processing loop
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Timer for data health monitoring
        self.health_timer = self.create_timer(self.health_check_period, self.health_check_callback)
        
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
            
        try:
            self.imu_data_subscription = self.create_subscription(
                Imu,
                f"/{self.robot_id}/imu/data",
                self.imu_data_callback,
                1)
        except Exception as e:
            self.get_logger().error(f"Failed to create IMU data subscription: {e}")

    def initialize_reference_gps(self):
        """Initialize reference GPS with retry mechanism"""
        def wait_for_service():
            retry_count = 0
            max_retries = 10
            while not self.cli.wait_for_service(timeout_sec=2.0) and retry_count < max_retries:
                self.get_logger().debug(f'Reference GPS service not available, waiting... (attempt {retry_count + 1})')
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
            self.get_logger().debug(f'[{self.req.robot_id}]Sent request for reference GPS')
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
            
            # Reset IMU-only pose estimation
            if self.imu_data is not None:
                # Reset position and velocity
                self.imu_pose_x = 0.0
                self.imu_pose_y = 0.0
                self.imu_velocity_x = 0.0
                self.imu_velocity_y = 0.0
                
                # Reset calibration
                q = self.imu_data.orientation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                self.imu_calibration = yaw
                
                # Reset drift compensation
                self.accel_offset_x = 0.0
                self.accel_offset_y = 0.0
                self.calibration_samples = []
                self.is_calibrating = True
                
                self.get_logger().info(f'[{self.robot_id}]IMU pose reset completed: calibration {self.imu_calibration:.2f}')
            else:
                self.get_logger().warn(f'[{self.robot_id}]IMU pose reset failed: missing IMU data')

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
    
    def imu_data_callback(self, msg):
        """Handle IMU data for pose estimation"""
        try:
            with self.data_lock:
                current_time = time.time()
                dt = current_time - self.last_imu_data_time
                
                # Skip if dt is too large (first message or after disconnection)
                if dt > 1.0:
                    self.last_imu_data_time = current_time
                    return
                
                # Store IMU data
                self.imu_data = msg
                
                # Extract orientation (theta)
                # Convert quaternion to yaw angle
                q = msg.orientation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                
                # Initialize calibration if not set
                if self.imu_calibration is None:
                    self.imu_calibration = yaw
                
                # Apply calibration offset
                self.imu_theta = yaw - self.imu_calibration
                
                # Normalize to [-π, π]
                while self.imu_theta > math.pi:
                    self.imu_theta -= 2 * math.pi
                while self.imu_theta < -math.pi:
                    self.imu_theta += 2 * math.pi
                
                # Extract linear acceleration (in body frame)
                accel_x = msg.linear_acceleration.x
                accel_y = msg.linear_acceleration.y
                
                # Perform initial calibration to determine acceleration offsets
                if self.is_calibrating:
                    self.calibration_samples.append((accel_x, accel_y))
                    if len(self.calibration_samples) >= self.calibration_sample_count:
                        # Calculate average acceleration as offset (assuming robot is stationary)
                        sum_x = sum(sample[0] for sample in self.calibration_samples)
                        sum_y = sum(sample[1] for sample in self.calibration_samples)
                        self.accel_offset_x = sum_x / len(self.calibration_samples)
                        self.accel_offset_y = sum_y / len(self.calibration_samples)
                        self.is_calibrating = False
                        self.get_logger().info(f'[{self.robot_id}]IMU acceleration calibrated: offset_x={self.accel_offset_x:.3f}, offset_y={self.accel_offset_y:.3f}')
                    return  # Skip pose estimation during calibration
                
                # Apply offset compensation
                corrected_accel_x = accel_x - self.accel_offset_x
                corrected_accel_y = accel_y - self.accel_offset_y
                
                # Apply threshold to reduce noise
                if abs(corrected_accel_x) < self.accel_threshold:
                    corrected_accel_x = 0.0
                if abs(corrected_accel_y) < self.accel_threshold:
                    corrected_accel_y = 0.0
                
                # Transform acceleration to world frame
                cos_theta = math.cos(yaw)
                sin_theta = math.sin(yaw)
                
                world_accel_x = corrected_accel_x * cos_theta - corrected_accel_y * sin_theta
                world_accel_y = corrected_accel_x * sin_theta + corrected_accel_y * cos_theta
                
                # Integrate acceleration to get velocity
                self.imu_velocity_x += world_accel_x * dt
                self.imu_velocity_y += world_accel_y * dt
                
                # Apply velocity decay to prevent drift accumulation
                self.imu_velocity_x *= self.velocity_decay_factor
                self.imu_velocity_y *= self.velocity_decay_factor
                
                # Integrate velocity to get position
                self.imu_pose_x += self.imu_velocity_x * dt
                self.imu_pose_y += self.imu_velocity_y * dt
                
                # Update timestamp
                self.last_imu_data_time = current_time
                
                # Publish IMU-only pose
                self.publish_imu_pose()
                
        except Exception as e:
            self.get_logger().error(f'IMU data callback error: {e}')
    
    def publish_imu_pose(self):
        """Publish IMU-only pose estimate"""
        try:
            msg = Pose2D()
            msg.x = self.imu_pose_x
            msg.y = self.imu_pose_y
            msg.theta = self.imu_theta
            
            self.imu_pose_publisher.publish(msg)
            self.get_logger().debug(f"Published IMU pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")
            
        except Exception as e:
            self.get_logger().error(f'IMU pose publish error: {e}')

    def health_check_callback(self):
        """Monitor sensor health and data availability"""
        current_time = time.time()
        
        # Check GPS health
        if current_time - self.last_gps_time > 5.0:  # 5 seconds timeout
            self.gps_data_available = False
            self.get_logger().debug('GPS data timeout detected')
        
        # Check IMU health
        if current_time - self.last_imu_time > 5.0:  # 5 seconds timeout
            self.imu_data_available = False
            self.get_logger().debug('IMU data timeout detected')
        
        # Check reference GPS
        if self.ref_lat is None or self.ref_lon is None:
            self.get_logger().debug('Reference GPS not available, retrying...')
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