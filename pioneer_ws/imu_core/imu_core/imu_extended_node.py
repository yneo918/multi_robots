import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Vector3, Twist, Accel, Quaternion
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray, Int16, Int16MultiArray
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

import time
import board
import adafruit_bno055
import os
import threading
import numpy as np
import yaml


class ExtendedImuNode(Node):
    def __init__(self):
        # Get robot ID from parameter or environment
        robot_id = os.getenv("ROBOT_ID", "pX")
        username = os.getenv("USER", "pioneer")
        
        super().__init__(f'{robot_id}_imu_extended')
        
        # Declare parameters with defaults from system_config.yaml structure
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', robot_id),
                ('heading_offset', 0),
                ('publish_rate', 20.0),  # Hz
                ('connection_timeout', 10.0),
                ('retry_delay', 2.0),
                ('calibration_file', f"/home/{username}/imu_calib/bno055_offsets.json"),
                ('calibFileLoc', f"/home/{username}/imu_calib/bno055_offsets.json"),
                ('accel_threshold', 0.1)  # m/s^2 - threshold for acceleration noise
            ]
        )
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.heading_offset = self.get_parameter('heading_offset').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.connection_timeout = self.get_parameter('connection_timeout').get_parameter_value().double_value
        self.retry_delay = self.get_parameter('retry_delay').get_parameter_value().double_value
        self.calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.accel_threshold = self.get_parameter('accel_threshold').get_parameter_value().double_value
        
        # Log loaded configuration
        self.get_logger().info(f'Extended IMU Node Configuration:')
        self.get_logger().info(f'  Robot ID: {self.robot_id}')
        self.get_logger().info(f'  Heading Offset: {self.heading_offset}°')
        self.get_logger().info(f'  Publish Rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Connection Timeout: {self.connection_timeout}s')
        self.get_logger().info(f'  Retry Delay: {self.retry_delay}s')
        self.get_logger().info(f'  Acceleration Threshold: {self.accel_threshold} m/s²')
        
        # Connection state
        self.i2c = None
        self.sensor = None
        self.connection_active = False
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Error handling
        self.max_retries = 5
        self.last_successful_read = time.time()
        
        # Create publishers for extended IMU data
        self.publisher_imu = self.create_publisher(
            Imu, 
            f'{self.robot_id}/imu/data', 
            10
        )
        
        # Create publishers for backward compatibility with original imu_node
        self.publisher_quaternion = self.create_publisher(
            Quaternion, 
            f'{self.robot_id}/imu/quaternion', 
            1
        )
        self.publisher_euler = self.create_publisher(
            Float32MultiArray, 
            f'{self.robot_id}/imu/eulerAngle', 
            3
        )
        self.publisher_calib = self.create_publisher(
            Int16MultiArray, 
            f'{self.robot_id}/imu/calibInfo', 
            4
        )
        
        self.publisher_mag = self.create_publisher(
            MagneticField, 
            f'{self.robot_id}/imu/magnetic_field', 
            10
        )
        
        self.publisher_linear_accel = self.create_publisher(
            Vector3, 
            f'{self.robot_id}/imu/linear_acceleration', 
            10
        )
        
        self.publisher_angular_vel = self.create_publisher(
            Vector3, 
            f'{self.robot_id}/imu/angular_velocity', 
            10
        )
        
        self.publisher_gravity = self.create_publisher(
            Vector3, 
            f'{self.robot_id}/imu/gravity_vector', 
            10
        )
        
        self.publisher_temperature = self.create_publisher(
            Int16, 
            f'{self.robot_id}/imu/temperature', 
            10
        )
        
        self.publisher_diagnostics = self.create_publisher(
            DiagnosticArray, 
            f'{self.robot_id}/imu/diagnostics', 
            10
        )
        
        # Create subscription for calibration commands (backward compatibility)
        self.subscription = self.create_subscription(
            Int16MultiArray,
            f'{self.robot_id}/imu/calibCom',
            self.calib_cmd_callback, 
            5
        )
        
        # Initialize IMU connection
        self.initialize_imu_connection()
        
        # Main timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Diagnostics timer
        self.diag_timer = self.create_timer(1.0, self.diagnostics_callback)
        
        # Health monitoring timer
        health_check_period = 5.0  # seconds
        self.health_timer = self.create_timer(health_check_period, self.health_check_callback)
        
        self.get_logger().info('Extended IMU node initialized')

    def initialize_imu_connection(self):
        """Initialize IMU connection with retry mechanism"""
        retry_count = 0
        max_retries = 5
        
        while retry_count < max_retries:
            if self.connect_to_imu():
                self.get_logger().info('IMU connected successfully')
                return
            
            retry_count += 1
            self.get_logger().warn(f'IMU connection attempt {retry_count} failed, retrying in {self.retry_delay}s')
            time.sleep(self.retry_delay)
        
        self.get_logger().error('Failed to connect to IMU after maximum retries')

    def connect_to_imu(self):
        """Establish connection to BNO055 IMU"""
        try:
            with self.data_lock:
                self.i2c = board.I2C()
                self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
                
                # Wait for sensor to be ready
                start_time = time.time()
                while time.time() - start_time < self.connection_timeout:
                    if self.sensor.calibration_status[0] >= 0:  # Valid status
                        self.connection_active = True
                        self.load_calibration()
                        return True
                    time.sleep(0.1)
                
                return False
                
        except Exception as e:
            self.get_logger().error(f'IMU connection error: {e}')
            return False

    def load_calibration(self):
        """Load calibration from file if available (supports both formats)"""
        try:
            calib_file = self.calibration_file
            
            if not os.path.exists(calib_file):
                # Try alternative path from parameter
                alt_calib_file = self.get_parameter('calibFileLoc').get_parameter_value().string_value
                if os.path.exists(alt_calib_file):
                    calib_file = alt_calib_file
                else:
                    self.get_logger().info('No calibration file found, using defaults')
                    return
            
            # Read calibration data
            with open(calib_file, 'r') as f:
                data = yaml.safe_load(f)
            
            # Try new format first (individual offsets/radius)
            if all(key in data for key in ['offsets_accelerometer', 'radius_accelerometer',
                                           'offsets_magnetometer', 'radius_magnetometer',
                                           'offsets_gyroscope']):
                self.get_logger().info('Loading calibration in new format')
                
                # Switch to CONFIG_MODE to allow writing registers
                self.sensor._write_register(adafruit_bno055._MODE_REGISTER,
                                          adafruit_bno055.CONFIG_MODE)
                time.sleep(0.02)  # per datasheet
                
                # Apply offsets via high-level properties
                self.sensor.offsets_accelerometer = tuple(data['offsets_accelerometer'])
                self.sensor.radius_accelerometer = data['radius_accelerometer']
                self.sensor.offsets_magnetometer = tuple(data['offsets_magnetometer'])
                self.sensor.radius_magnetometer = data['radius_magnetometer']
                self.sensor.offsets_gyroscope = tuple(data['offsets_gyroscope'])
                
                # Return to NDOF_MODE for normal fusion operation
                time.sleep(0.01)
                self.sensor._write_register(adafruit_bno055._MODE_REGISTER,
                                          adafruit_bno055.NDOF_MODE)
                
                self.get_logger().info('Calibration loaded and applied from file (new format)')
                return
            
            # Try legacy format (22-byte array)
            robot_data = data.get(f'{self.robot_id}_imu', {})
            params = robot_data.get('ros__parameters', {})
            calib_data = params.get('calibOffsetsRadii', [])
            
            if calib_data and len(calib_data) == 22:
                self.get_logger().info('Loading calibration in legacy format')
                self.apply_calibration_data(calib_data)
                return
            elif calib_data:
                self.get_logger().warn(f'Invalid calibration data length: {len(calib_data)}, expected 22')
                return
            
            # If no valid format found, log error
            self.get_logger().error('Calibration file format not recognized')
                    
        except Exception as e:
            self.get_logger().warn(f'Failed to load calibration: {e}')

    def timer_callback(self):
        """Main callback to publish all IMU data"""
        try:
            with self.data_lock:
                if not self.connection_active or not self.sensor:
                    return
                
                # Get current timestamp
                stamp = self.get_clock().now().to_msg()
                
                # Publish combined IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = stamp
                imu_msg.header.frame_id = f"{self.robot_id}_imu_link"
                
                # Orientation (quaternion)
                quat = self.sensor.quaternion
                if quat and None not in quat:
                    imu_msg.orientation.w = quat[0]
                    imu_msg.orientation.x = quat[1]
                    imu_msg.orientation.y = quat[2]
                    imu_msg.orientation.z = quat[3]
                    imu_msg.orientation_covariance[0] = 0.01  # Variance
                    imu_msg.orientation_covariance[4] = 0.01
                    imu_msg.orientation_covariance[8] = 0.01
                    
                    # Publish quaternion for backward compatibility
                    quat_msg = Quaternion()
                    quat_msg.w = float(quat[0])
                    quat_msg.x = float(quat[1])
                    quat_msg.y = float(quat[2])
                    quat_msg.z = float(quat[3])
                    self.publisher_quaternion.publish(quat_msg)
                
                # Angular velocity (rad/s)
                gyro = self.sensor.gyro
                if gyro and None not in gyro:
                    imu_msg.angular_velocity.x = gyro[0]
                    imu_msg.angular_velocity.y = gyro[1]
                    imu_msg.angular_velocity.z = gyro[2]
                    imu_msg.angular_velocity_covariance[0] = 0.01
                    imu_msg.angular_velocity_covariance[4] = 0.01
                    imu_msg.angular_velocity_covariance[8] = 0.01
                
                # Linear acceleration (m/s^2) with threshold filtering
                linear_accel = self.sensor.linear_acceleration
                if linear_accel and None not in linear_accel:
                    # Apply threshold to filter out noise
                    ax = linear_accel[0] if abs(linear_accel[0]) >= self.accel_threshold else 0.0
                    ay = linear_accel[1] if abs(linear_accel[1]) >= self.accel_threshold else 0.0
                    az = linear_accel[2] if abs(linear_accel[2]) >= self.accel_threshold else 0.0
                    
                    imu_msg.linear_acceleration.x = ax
                    imu_msg.linear_acceleration.y = ay
                    imu_msg.linear_acceleration.z = az
                    imu_msg.linear_acceleration_covariance[0] = 0.1
                    imu_msg.linear_acceleration_covariance[4] = 0.1
                    imu_msg.linear_acceleration_covariance[8] = 0.1
                
                self.publisher_imu.publish(imu_msg)
                
                # Publish individual messages
                # Linear acceleration (with threshold applied)
                if linear_accel and None not in linear_accel:
                    linear_msg = Vector3()
                    linear_msg.x = ax  # Use threshold-filtered values
                    linear_msg.y = ay
                    linear_msg.z = az
                    self.publisher_linear_accel.publish(linear_msg)
                
                # Angular velocity
                if gyro and None not in gyro:
                    angular_msg = Vector3()
                    angular_msg.x = gyro[0]
                    angular_msg.y = gyro[1]
                    angular_msg.z = gyro[2]
                    self.publisher_angular_vel.publish(angular_msg)
                
                # Gravity vector
                gravity = self.sensor.gravity
                if gravity and None not in gravity:
                    gravity_msg = Vector3()
                    gravity_msg.x = gravity[0]
                    gravity_msg.y = gravity[1]
                    gravity_msg.z = gravity[2]
                    self.publisher_gravity.publish(gravity_msg)
                
                # Magnetic field
                mag = self.sensor.magnetic
                if mag and None not in mag:
                    mag_msg = MagneticField()
                    mag_msg.header.stamp = stamp
                    mag_msg.header.frame_id = f"{self.robot_id}_imu_link"
                    mag_msg.magnetic_field.x = mag[0] * 1e-6  # Convert to Tesla
                    mag_msg.magnetic_field.y = mag[1] * 1e-6
                    mag_msg.magnetic_field.z = mag[2] * 1e-6
                    mag_msg.magnetic_field_covariance[0] = 1e-7
                    mag_msg.magnetic_field_covariance[4] = 1e-7
                    mag_msg.magnetic_field_covariance[8] = 1e-7
                    self.publisher_mag.publish(mag_msg)
                
                # Temperature
                temp = self.sensor.temperature
                if temp is not None:
                    temp_msg = Int16()
                    temp_msg.data = int(temp)
                    self.publisher_temperature.publish(temp_msg)
                
                # Euler angles for backward compatibility
                euler = self.sensor.euler
                if euler and None not in euler:
                    euler_msg = Float32MultiArray()
                    euler_msg.data = [float(euler[0]), float(euler[1]), float(euler[2])]
                    # Apply heading offset from parameters
                    euler_msg.data[0] = (euler_msg.data[0] + self.heading_offset) % 360
                    self.publisher_euler.publish(euler_msg)
                
                # Calibration status for backward compatibility
                calib_status = self.sensor.calibration_status
                if calib_status:
                    calib_msg = Int16MultiArray()
                    calib_msg.data = [int(status) for status in calib_status]
                    self.publisher_calib.publish(calib_msg)
                
                # Update last successful read time
                self.last_successful_read = time.time()
                    
        except Exception as e:
            self.get_logger().error(f'Timer callback error: {e}')
            self.handle_sensor_error()

    def diagnostics_callback(self):
        """Publish diagnostic information"""
        try:
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            status = DiagnosticStatus()
            status.name = f"{self.robot_id}_imu_extended"
            status.hardware_id = f"{self.robot_id}_bno055"
            
            with self.data_lock:
                if self.connection_active and self.sensor:
                    # Get calibration status
                    cal_status = self.sensor.calibration_status
                    
                    # Overall status
                    if all(s >= 2 for s in cal_status):
                        status.level = DiagnosticStatus.OK
                        status.message = "IMU fully calibrated and operational"
                    elif any(s >= 1 for s in cal_status):
                        status.level = DiagnosticStatus.WARN
                        status.message = "IMU partially calibrated"
                    else:
                        status.level = DiagnosticStatus.ERROR
                        status.message = "IMU not calibrated"
                    
                    # Add calibration details
                    status.values.append(KeyValue(key="System Calibration", value=str(cal_status[0])))
                    status.values.append(KeyValue(key="Gyro Calibration", value=str(cal_status[1])))
                    status.values.append(KeyValue(key="Accel Calibration", value=str(cal_status[2])))
                    status.values.append(KeyValue(key="Mag Calibration", value=str(cal_status[3])))
                    
                    # Add temperature
                    temp = self.sensor.temperature
                    if temp is not None:
                        status.values.append(KeyValue(key="Temperature", value=f"{temp}°C"))
                    
                else:
                    status.level = DiagnosticStatus.ERROR
                    status.message = "IMU not connected"
                
            diag_array.status.append(status)
            self.publisher_diagnostics.publish(diag_array)
            
        except Exception as e:
            self.get_logger().error(f'Diagnostics error: {e}')

    def handle_sensor_error(self):
        """Handle sensor errors by attempting reconnection"""
        with self.data_lock:
            self.connection_active = False
        
        # Attempt reconnection in a separate thread
        threading.Thread(target=self.reconnect_imu, daemon=True).start()

    def reconnect_imu(self):
        """Reconnect to IMU after error"""
        self.get_logger().warn('Attempting to reconnect to IMU...')
        time.sleep(self.retry_delay)
        self.initialize_imu_connection()
    
    def calib_cmd_callback(self, msg: Int16MultiArray):
        """Handle calibration commands with error handling (backward compatibility)"""
        try:
            if len(msg.data) < 3:
                self.get_logger().warn('Invalid calibration command message')
                return
                
            run_calib_reset_device = msg.data[0]
            set_calib_param = msg.data[1]
            store_calib = msg.data[2]

            if run_calib_reset_device:
                self.get_logger().info('Resetting IMU device')
                threading.Thread(target=self.reset_imu_device, daemon=True).start()
                
            elif set_calib_param:
                self.get_logger().info('Setting calibration from parameters')
                self.set_calibration_from_params()
                
            elif store_calib:
                self.get_logger().info('Storing current calibration')
                self.store_calibration()
                
        except Exception as e:
            self.get_logger().error(f'Calibration command error: {e}')
    
    def reset_imu_device(self):
        """Reset IMU device by reconnecting"""
        try:
            with self.data_lock:
                self.connection_active = False
            
            time.sleep(1)  # Brief pause
            self.initialize_imu_connection()
            
        except Exception as e:
            self.get_logger().error(f'IMU reset error: {e}')
    
    def set_calibration_from_params(self):
        """Set calibration from ROS parameters"""
        try:
            calib_data = self.get_parameter('calibOffsetsRadii').get_parameter_value().integer_array_value
            self.apply_calibration_data(calib_data)
        except Exception as e:
            self.get_logger().error(f'Failed to set calibration from parameters: {e}')
    
    def store_calibration(self):
        """Store current calibration data in both formats"""
        try:
            with self.data_lock:
                if not self.connection_active or not self.sensor:
                    self.get_logger().warn('Cannot store calibration: IMU not connected')
                    return
                
                # Get calibration data in both formats
                calib_data_legacy = self.get_calibration()
                calib_data_new = self.get_calibration_new_format()
            
            calib_file = self.calibration_file
            
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(calib_file), exist_ok=True)
            
            # Create data structure with both formats
            data = {
                # New format (preferred)
                'offsets_accelerometer': calib_data_new['offsets_accelerometer'],
                'radius_accelerometer': calib_data_new['radius_accelerometer'],
                'offsets_magnetometer': calib_data_new['offsets_magnetometer'],
                'radius_magnetometer': calib_data_new['radius_magnetometer'],
                'offsets_gyroscope': calib_data_new['offsets_gyroscope'],
                
                # Legacy format for backward compatibility
                f'{self.robot_id}_imu': {
                    'ros__parameters': {
                        'calibrateIMU': 1, 
                        'calibOffsetsRadii': calib_data_legacy,
                        'calibFileLoc': calib_file
                    }
                }
            }

            with open(calib_file, 'w') as yaml_file:
                yaml.dump(data, yaml_file, default_flow_style=None)
                
            self.get_logger().info(f'Calibration stored to {calib_file}')
             
        except Exception as e:
            self.get_logger().error(f'Failed to store calibration: {e}')
    
    def get_calibration(self):
        """Get current calibration data from sensor (legacy format)"""
        try:
            # Switch to configuration mode
            self.sensor._write_register(adafruit_bno055._MODE_REGISTER, adafruit_bno055.CONFIG_MODE)
            time.sleep(0.02)

            # Read the 22 bytes of calibration data
            calib_data = []
            register_addr = 0x55  # Start with highest register
            
            for _ in range(22):
                calib_data.append(self.sensor._read_register(register_addr))
                register_addr -= 1

            # Return to normal operation mode
            time.sleep(0.01)
            self.sensor._write_register(adafruit_bno055._MODE_REGISTER, adafruit_bno055.NDOF_MODE)
            
            return calib_data
            
        except Exception as e:
            self.get_logger().error(f'Failed to get calibration data: {e}')
            # Try to return to normal mode
            try:
                self.sensor._write_register(adafruit_bno055._MODE_REGISTER, adafruit_bno055.NDOF_MODE)
            except:
                pass
            return []
    
    def get_calibration_new_format(self):
        """Get current calibration data in new format"""
        try:
            # Switch to configuration mode
            self.sensor._write_register(adafruit_bno055._MODE_REGISTER, adafruit_bno055.CONFIG_MODE)
            time.sleep(0.02)
            
            # Read calibration values using high-level properties
            calib_data = {
                'offsets_accelerometer': list(self.sensor.offsets_accelerometer),
                'radius_accelerometer': self.sensor.radius_accelerometer,
                'offsets_magnetometer': list(self.sensor.offsets_magnetometer),
                'radius_magnetometer': self.sensor.radius_magnetometer,
                'offsets_gyroscope': list(self.sensor.offsets_gyroscope)
            }
            
            # Return to normal operation mode
            time.sleep(0.01)
            self.sensor._write_register(adafruit_bno055._MODE_REGISTER, adafruit_bno055.NDOF_MODE)
            
            return calib_data
            
        except Exception as e:
            self.get_logger().error(f'Failed to get calibration data (new format): {e}')
            # Try to return to normal mode
            try:
                self.sensor._write_register(adafruit_bno055._MODE_REGISTER, adafruit_bno055.NDOF_MODE)
            except:
                pass
            return {
                'offsets_accelerometer': [0, 0, 0],
                'radius_accelerometer': 0,
                'offsets_magnetometer': [0, 0, 0],
                'radius_magnetometer': 0,
                'offsets_gyroscope': [0, 0, 0]
            }
    
    def health_check_callback(self):
        """Monitor IMU connection health"""
        current_time = time.time()
        
        if current_time - self.last_successful_read > self.connection_timeout:
            self.get_logger().warn('IMU connection timeout detected')
            with self.data_lock:
                self.connection_active = False
            
            # Attempt reconnection in a separate thread
            threading.Thread(target=self.reconnect_imu, daemon=True).start()
    
    def apply_calibration_data(self, calib_data):
        """Apply calibration data to sensor (legacy format)"""
        try:
            # Switch to CONFIG_MODE
            self.sensor._write_register(adafruit_bno055._MODE_REGISTER, adafruit_bno055.CONFIG_MODE)
            time.sleep(0.02)
            
            # Write calibration data (22 bytes starting from register 0x55 going down to 0x40)
            register_addr = 0x55
            for byte_val in calib_data:
                self.sensor._write_register(register_addr, byte_val)
                register_addr -= 1
            
            # Return to NDOF_MODE
            time.sleep(0.01)
            self.sensor._write_register(adafruit_bno055._MODE_REGISTER, adafruit_bno055.NDOF_MODE)
            time.sleep(0.1)  # Allow mode change to complete
            
        except Exception as e:
            self.get_logger().error(f'Failed to apply calibration data: {e}')
            # Try to return to normal mode
            try:
                self.sensor._write_register(adafruit_bno055._MODE_REGISTER, adafruit_bno055.NDOF_MODE)
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ExtendedImuNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Extended IMU node failed: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()