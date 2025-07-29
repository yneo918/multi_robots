import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Quaternion, Vector3, Twist, Accel
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray, Int16, Int16MultiArray
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

import time
import board
import adafruit_bno055
import yaml
import os
import threading
import numpy as np

# Register address constants
BNO055_CALIB_START_REGISTER = 0x55
BNO055_CALIB_END_REGISTER = 0x40
BNO055_CALIB_DATA_SIZE = 22


class ReadImu(Node):
    """
    Unified IMU Node that combines functionality from both imu_node.py and imu_extended_node.py.
    
    Features:
    - Standard sensor_msgs/Imu message publishing
    - Legacy compatibility messages (quaternion, euler angles)
    - Extended sensor data (magnetic field, gravity, temperature)
    - Dual calibration format support (new and legacy)
    - Configurable output modes
    - Robust error handling and reconnection
    - Health monitoring and diagnostics
    """
    
    def __init__(self):
        # Get robot ID from parameter or environment
        robot_id = os.getenv("ROBOT_ID", "pX")
        username = os.getenv("USER", "pioneer")
        
        super().__init__(f'{robot_id}_imu')
        
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
                ('enable_extended_output', True),  # Enable extended data outputs
                ('enable_legacy_compatibility', True)  # Enable legacy message formats
            ]
        )
        
        # Get parameters with validation
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        if not self.robot_id or not self.robot_id.strip():
            raise ValueError("robot_id parameter cannot be empty")
        
        self.heading_offset = self.get_parameter('heading_offset').get_parameter_value().integer_value
        if not -360 <= self.heading_offset <= 360:
            raise ValueError(f"heading_offset must be between -360 and 360 degrees, got {self.heading_offset}")
        
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        if self.publish_rate <= 0 or self.publish_rate > 1000:
            raise ValueError(f"publish_rate must be between 0 and 1000 Hz, got {self.publish_rate}")
        
        self.connection_timeout = self.get_parameter('connection_timeout').get_parameter_value().double_value
        if self.connection_timeout <= 0:
            raise ValueError(f"connection_timeout must be positive, got {self.connection_timeout}")
        
        self.retry_delay = self.get_parameter('retry_delay').get_parameter_value().double_value
        if self.retry_delay <= 0:
            raise ValueError(f"retry_delay must be positive, got {self.retry_delay}")
        
        self.calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        if not self.calibration_file or not self.calibration_file.strip():
            self.get_logger().warn("calibration_file parameter is empty, using defaults")
        
        self.enable_extended_output = self.get_parameter('enable_extended_output').get_parameter_value().bool_value
        self.enable_legacy_compatibility = self.get_parameter('enable_legacy_compatibility').get_parameter_value().bool_value
        
        # Log loaded configuration
        self.get_logger().info(f'Unified IMU Node Configuration:')
        self.get_logger().info(f'  Robot ID: {self.robot_id}')
        self.get_logger().info(f'  Heading Offset: {self.heading_offset}°')
        self.get_logger().info(f'  Publish Rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Connection Timeout: {self.connection_timeout}s')
        self.get_logger().info(f'  Retry Delay: {self.retry_delay}s')
        self.get_logger().info(f'  Extended Output: {self.enable_extended_output}')
        self.get_logger().info(f'  Legacy Compatibility: {self.enable_legacy_compatibility}')
        
        # Connection state
        self.i2c = None
        self.sensor = None
        self.connection_active = False
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Error handling
        self.max_retries = 5
        self.last_successful_read = time.time()
        
        # Create publishers and subscriptions
        self.setup_publishers()
        self.setup_subscriptions()
        
        # Initialize IMU connection
        self.initialize_imu_connection()
        
        # Setup timers
        self.setup_timers()
        
        self.get_logger().info('Unified IMU node initialized')

    def setup_publishers(self):
        """Setup all publishers based on configuration"""
        # Standard IMU message (always published)
        self.publisher_imu = self.create_publisher(
            Imu, 
            f'{self.robot_id}/imu/data', 
            10
        )
        
        # Legacy compatibility publishers (for backward compatibility with existing code)
        if self.enable_legacy_compatibility:
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
        
        # Calibration info (always published for compatibility)
        self.publisher_calib = self.create_publisher(
            Int16MultiArray, 
            f'{self.robot_id}/imu/calibInfo', 
            4
        )
        
        # Extended output publishers (from imu_extended_node functionality)
        if self.enable_extended_output:
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

    def setup_subscriptions(self):
        """Setup subscriptions for calibration commands"""
        self.subscription = self.create_subscription(
            Int16MultiArray,
            f'{self.robot_id}/imu/calibCom',
            self.calib_cmd_callback, 
            5
        )

    def setup_timers(self):
        """Setup all timers"""
        # Main publishing timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Health monitoring timer
        health_check_period = 5.0  # seconds
        self.health_timer = self.create_timer(health_check_period, self.health_check_callback)
        
        # Diagnostics timer (only if extended output enabled)
        if self.enable_extended_output:
            self.diag_timer = self.create_timer(1.0, self.diagnostics_callback)

    def initialize_imu_connection(self):
        """Initialize IMU connection with retry mechanism"""
        retry_count = 0
        
        while retry_count < self.max_retries:
            if self.connect_to_imu():
                self.get_logger().info('IMU connected successfully')
                return
            
            retry_count += 1
            self.get_logger().warn(f'IMU connection attempt {retry_count} failed, retrying in {self.retry_delay}s')
            time.sleep(self.retry_delay)
        
        self.get_logger().error('Failed to connect to IMU after maximum retries')

    def connect_to_imu(self):
        """Attempt to connect to IMU"""
        try:
            self.get_logger().debug('Attempting to connect to IMU...')
            
            # Create I2C connection
            i2c = board.I2C()
            sensor = adafruit_bno055.BNO055_I2C(i2c)
            
            # Test the connection by reading sensor data
            time.sleep(1)  # Allow sensor to initialize
            _ = sensor.temperature  # Test read
            
            with self.data_lock:
                self.i2c = i2c
                self.sensor = sensor
                self.connection_active = True
                self.last_successful_read = time.time()
            
            # Load calibration if available
            self.load_calibration()
            
            return True
            
        except Exception as e:
            self.get_logger().warn(f'Failed to connect to IMU: {e}')
            return False

    def reconnect_imu(self):
        """Attempt to reconnect IMU"""
        self.get_logger().warn('Attempting to reconnect IMU...')
        
        with self.data_lock:
            self.i2c = None
            self.sensor = None
            self.connection_active = False
        
        # Try to reconnect
        self.initialize_imu_connection()

    def health_check_callback(self):
        """Monitor IMU connection health"""
        current_time = time.time()
        
        if current_time - self.last_successful_read > self.connection_timeout:
            self.get_logger().warn('IMU connection timeout detected')
            with self.data_lock:
                self.connection_active = False
            
            # Attempt reconnection in a separate thread
            threading.Thread(target=self.reconnect_imu, daemon=True).start()

    def timer_callback(self):
        """Main IMU reading callback with error handling"""
        try:
            with self.data_lock:
                if not self.connection_active or not self.sensor:
                    return
                
                # Get current timestamp
                stamp = self.get_clock().now().to_msg()
                
                # Read all sensor data in one batch to minimize I2C transactions
                sensor_data = self.read_sensor_data()
                
                if sensor_data:
                    # Publish standard IMU message (always)
                    self.publish_imu_message(sensor_data, stamp)
                    
                    # Publish legacy compatibility messages (if enabled)
                    if self.enable_legacy_compatibility:
                        self.publish_legacy_messages(sensor_data)
                    
                    # Publish extended output messages (if enabled)
                    if self.enable_extended_output:
                        self.publish_extended_messages(sensor_data, stamp)
                    
                    # Always publish calibration info for compatibility
                    self.publish_calibration_info(sensor_data)
                    
                    # Update last successful read time
                    self.last_successful_read = time.time()
                    
        except Exception as e:
            self.get_logger().error(f'IMU timer callback error: {e}')
            self.handle_sensor_error()

    def read_sensor_data(self):
        """Read all sensor data in one go to minimize I2C transactions"""
        try:
            data = {}
            
            # Read all available data
            data['quaternion'] = self.sensor.quaternion
            data['euler'] = self.sensor.euler
            data['gyro'] = self.sensor.gyro
            data['linear_acceleration'] = self.sensor.linear_acceleration
            data['acceleration'] = self.sensor.acceleration
            data['gravity'] = self.sensor.gravity
            data['magnetic'] = self.sensor.magnetic
            data['temperature'] = self.sensor.temperature
            data['calibration_status'] = self.sensor.calibration_status
            
            return data
            
        except Exception as e:
            self.get_logger().error(f'Failed to read sensor data: {e}')
            return None

    def publish_imu_message(self, sensor_data, stamp):
        """Publish standard sensor_msgs/Imu message"""
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = f"{self.robot_id}_imu_link"
        
        # Orientation (quaternion)
        quaternion = sensor_data.get('quaternion')
        if quaternion and all(q is not None for q in quaternion):
            imu_msg.orientation.x = float(quaternion[0])
            imu_msg.orientation.y = float(quaternion[1])
            imu_msg.orientation.z = float(quaternion[2])
            imu_msg.orientation.w = float(quaternion[3])
            # Set orientation covariance
            imu_msg.orientation_covariance[0] = 0.01
            imu_msg.orientation_covariance[4] = 0.01
            imu_msg.orientation_covariance[8] = 0.01
        
        # Angular velocity (rad/s)
        gyro = sensor_data.get('gyro')
        if gyro and all(g is not None for g in gyro):
            # Convert from degrees/sec to rad/sec
            imu_msg.angular_velocity.x = float(np.radians(gyro[0]))
            imu_msg.angular_velocity.y = float(np.radians(gyro[1]))
            imu_msg.angular_velocity.z = float(np.radians(gyro[2]))
            imu_msg.angular_velocity_covariance[0] = 0.01
            imu_msg.angular_velocity_covariance[4] = 0.01
            imu_msg.angular_velocity_covariance[8] = 0.01
        
        # Linear acceleration (m/s^2) - raw data without filtering
        linear_accel = sensor_data.get('linear_acceleration')
        if linear_accel and all(a is not None for a in linear_accel):
            imu_msg.linear_acceleration.x = float(linear_accel[0])
            imu_msg.linear_acceleration.y = float(linear_accel[1])
            imu_msg.linear_acceleration.z = float(linear_accel[2])
            imu_msg.linear_acceleration_covariance[0] = 0.1
            imu_msg.linear_acceleration_covariance[4] = 0.1
            imu_msg.linear_acceleration_covariance[8] = 0.1
        
        self.publisher_imu.publish(imu_msg)

    def publish_legacy_messages(self, sensor_data):
        """Publish legacy compatibility messages (from original imu_node.py)"""
        # Quaternion message
        quaternion = sensor_data.get('quaternion')
        if quaternion and all(q is not None for q in quaternion):
            msg_quat = Quaternion()
            msg_quat.x = float(quaternion[0])
            msg_quat.y = float(quaternion[1])
            msg_quat.z = float(quaternion[2])
            msg_quat.w = float(quaternion[3])
            self.publisher_quaternion.publish(msg_quat)

        # Euler angle message
        euler = sensor_data.get('euler')
        if euler and all(e is not None for e in euler):
            msg_euler = Float32MultiArray()
            msg_euler.data = [float(euler[0]), float(euler[1]), float(euler[2])]
            # Apply heading offset from parameters
            msg_euler.data[0] = (msg_euler.data[0] + self.heading_offset) % 360
            self.publisher_euler.publish(msg_euler)

    def publish_extended_messages(self, sensor_data, stamp):
        """Publish extended output messages (from imu_extended_node.py functionality)"""
        # Linear acceleration (raw data without filtering)
        linear_accel = sensor_data.get('linear_acceleration')
        if linear_accel and all(a is not None for a in linear_accel):
            linear_msg = Vector3()
            linear_msg.x = float(linear_accel[0])
            linear_msg.y = float(linear_accel[1])
            linear_msg.z = float(linear_accel[2])
            self.publisher_linear_accel.publish(linear_msg)
        
        # Angular velocity (in degrees/sec for compatibility)
        gyro = sensor_data.get('gyro')
        if gyro and all(g is not None for g in gyro):
            angular_msg = Vector3()
            angular_msg.x = float(gyro[0])
            angular_msg.y = float(gyro[1])
            angular_msg.z = float(gyro[2])
            self.publisher_angular_vel.publish(angular_msg)
        
        # Gravity vector
        gravity = sensor_data.get('gravity')
        if gravity and all(g is not None for g in gravity):
            gravity_msg = Vector3()
            gravity_msg.x = float(gravity[0])
            gravity_msg.y = float(gravity[1])
            gravity_msg.z = float(gravity[2])
            self.publisher_gravity.publish(gravity_msg)
        
        # Magnetic field
        magnetic = sensor_data.get('magnetic')
        if magnetic and all(m is not None for m in magnetic):
            mag_msg = MagneticField()
            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = f"{self.robot_id}_imu_link"
            mag_msg.magnetic_field.x = float(magnetic[0] * 1e-6)  # Convert to Tesla
            mag_msg.magnetic_field.y = float(magnetic[1] * 1e-6)
            mag_msg.magnetic_field.z = float(magnetic[2] * 1e-6)
            mag_msg.magnetic_field_covariance[0] = 1e-7
            mag_msg.magnetic_field_covariance[4] = 1e-7
            mag_msg.magnetic_field_covariance[8] = 1e-7
            self.publisher_mag.publish(mag_msg)
        
        # Temperature
        temperature = sensor_data.get('temperature')
        if temperature is not None:
            temp_msg = Int16()
            temp_msg.data = int(temperature)
            self.publisher_temperature.publish(temp_msg)

    def publish_calibration_info(self, sensor_data):
        """Publish calibration status (for compatibility)"""
        calib_status = sensor_data.get('calibration_status')
        if calib_status:
            calib_msg = Int16MultiArray()
            calib_msg.data = [int(status) for status in calib_status]
            self.publisher_calib.publish(calib_msg)

    def diagnostics_callback(self):
        """Publish diagnostic information"""
        try:
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            status = DiagnosticStatus()
            status.name = f"{self.robot_id}_imu"
            status.hardware_id = f"{self.robot_id}_bno055"
            
            with self.data_lock:
                if self.connection_active and self.sensor:
                    # Get calibration status
                    cal_status = self.sensor.calibration_status
                    
                    # Overall status based on calibration
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

    def calib_cmd_callback(self, msg: Int16MultiArray):
        """Handle calibration commands with error handling"""
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

    def load_calibration(self):
        """Load calibration from file if available (supports both formats)"""
        try:
            calib_file = self.calibration_file
            
            if not os.path.exists(calib_file):
                # Try alternative path from parameter
                try:
                    alt_calib_file = self.get_parameter('calibFileLoc').get_parameter_value().string_value
                    if os.path.exists(alt_calib_file):
                        calib_file = alt_calib_file
                    else:
                        self.get_logger().debug('No calibration file found, using defaults')
                        return
                except:
                    self.get_logger().debug('No calibration file found, using defaults')
                    return
            
            # Read calibration data
            with open(calib_file, 'r') as f:
                data = yaml.safe_load(f)
            
            # Try new format first (individual offsets/radius)
            if all(key in data for key in ['offsets_accelerometer', 'radius_accelerometer',
                                           'offsets_magnetometer', 'radius_magnetometer',
                                           'offsets_gyroscope']):
                self.get_logger().debug('Loading calibration in new format')
                self.apply_calibration_new_format(data)
                return
            
            # Try legacy format (22-byte array)
            robot_data = data.get(f'{self.robot_id}_imu', {})
            params = robot_data.get('ros__parameters', {})
            calib_data = params.get('calibOffsetsRadii', [])
            
            if calib_data and len(calib_data) == BNO055_CALIB_DATA_SIZE:
                self.get_logger().debug('Loading calibration in legacy format')
                self.apply_calibration_data(calib_data)
                return
            elif calib_data:
                self.get_logger().warn(f'Invalid calibration data length: {len(calib_data)}, expected {BNO055_CALIB_DATA_SIZE}')
                return
            
            # If no valid format found, log warning
            self.get_logger().warn('Calibration file format not recognized')
                    
        except Exception as e:
            self.get_logger().warn(f'Failed to load calibration: {e}')

    def apply_calibration_new_format(self, data):
        """Apply calibration using new format with individual offsets/radius"""
        try:
            # Extract calibration values
            accel_offsets = tuple(data['offsets_accelerometer'])
            accel_radius = data['radius_accelerometer']
            mag_offsets = tuple(data['offsets_magnetometer'])
            mag_radius = data['radius_magnetometer']
            gyro_offsets = tuple(data['offsets_gyroscope'])
            
            # Switch to CONFIG_MODE to allow writing registers
            self.sensor._write_register(adafruit_bno055._MODE_REGISTER,
                                      adafruit_bno055.CONFIG_MODE)
            time.sleep(0.02)  # per datasheet
            
            # Apply offsets via high-level properties
            self.sensor.offsets_accelerometer = accel_offsets
            self.sensor.radius_accelerometer = accel_radius
            self.sensor.offsets_magnetometer = mag_offsets
            self.sensor.radius_magnetometer = mag_radius
            self.sensor.offsets_gyroscope = gyro_offsets
            
            # Return to NDOF_MODE for normal fusion operation
            time.sleep(0.01)
            self.sensor._write_register(adafruit_bno055._MODE_REGISTER,
                                      adafruit_bno055.NDOF_MODE)
            
            self.get_logger().debug('Calibration loaded and applied from file (new format)')
            
        except Exception as e:
            self.get_logger().error(f'Failed to apply calibration (new format): {e}')
            # Try to return to normal mode
            try:
                self.sensor._write_register(adafruit_bno055._MODE_REGISTER, adafruit_bno055.NDOF_MODE)
            except:
                pass

    def apply_calibration_data(self, calib_data):
        """Apply calibration data to sensor (legacy format)"""
        try:
            # Switch to CONFIG_MODE
            self.sensor._write_register(adafruit_bno055._MODE_REGISTER, adafruit_bno055.CONFIG_MODE)
            time.sleep(0.02)
            
            # Write calibration data (22 bytes starting from register 0x55 going down to 0x40)
            register_addr = BNO055_CALIB_START_REGISTER
            for byte_val in calib_data:
                self.sensor._write_register(register_addr, byte_val)
                register_addr -= 1
            
            # Return to NDOF_MODE
            time.sleep(0.01)
            self.sensor._write_register(adafruit_bno055._MODE_REGISTER, adafruit_bno055.NDOF_MODE)
            time.sleep(0.1)  # Allow mode change to complete
            
            self.get_logger().debug('Calibration loaded and applied from file (legacy format)')
            
        except Exception as e:
            self.get_logger().error(f'Failed to apply calibration data: {e}')
            # Try to return to normal mode
            try:
                self.sensor._write_register(adafruit_bno055._MODE_REGISTER, adafruit_bno055.NDOF_MODE)
            except:
                pass

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
            register_addr = BNO055_CALIB_START_REGISTER  # Start with highest register
            
            for _ in range(BNO055_CALIB_DATA_SIZE):
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

    def destroy_node(self):
        """Clean up resources when destroying node"""
        with self.data_lock:
            self.connection_active = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    retry_count = 0
    max_retries = 3
    
    while retry_count < max_retries:
        try:
            get_imu_data = ReadImu()
            rclpy.spin(get_imu_data)
            break  # If we get here, normal shutdown occurred
            
        except Exception as e:
            retry_count += 1
            print(f"IMU node failed (attempt {retry_count}): {e}")
            
            if retry_count < max_retries:
                print(f"Retrying in 5 seconds...")
                time.sleep(5)
            else:
                print("IMU node failed after maximum retries")
                
        finally:
            try:
                get_imu_data.destroy_node()
            except:
                pass
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()