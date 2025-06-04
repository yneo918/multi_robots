import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray

import time
import board
import adafruit_bno055
import yaml
import os
import threading


class ReadImu(Node):
    def __init__(self):
        self.robot_id = os.getenv("ROBOT_ID")
        self.username = os.getenv("USER")
        super().__init__(f'{self.robot_id}_imu')

        # IMU configuration
        imu_offset = os.getenv("IMU_OFFSET")
        self.heading_offset = int(imu_offset) if imu_offset else 0
        
        # Connection state
        self.i2c = None
        self.sensor = None
        self.connection_active = False
        
        # Error handling
        self.max_retries = 5
        self.retry_delay = 2.0
        self.last_successful_read = time.time()
        self.connection_timeout = 10.0  # seconds
        
        # Thread safety
        self.data_lock = threading.Lock()

        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('calibFileLoc', f"/home/{self.username}/imu_calib/bno055_offsets.json")
            ]
        )
        
        # Create publishers
        self.publisher_quaternion = self.create_publisher(Quaternion, f'{self.robot_id}/imu/quaternion', 1)
        self.publisher_euler = self.create_publisher(Float32MultiArray, f'{self.robot_id}/imu/eulerAngle', 3)
        self.publisher_calib = self.create_publisher(Int16MultiArray, f'{self.robot_id}/imu/calibInfo', 4)

        # Create subscription for calibration commands
        self.subscription = self.create_subscription(
            Int16MultiArray,
            f'{self.robot_id}/imu/calibCom',
            self.calib_cmd_callback, 
            5)

        # Initialize IMU connection
        self.initialize_imu_connection()

        # Timers
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Health monitoring timer
        health_check_period = 5.0  # seconds
        self.health_timer = self.create_timer(health_check_period, self.health_check_callback)

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
            self.get_logger().info('Attempting to connect to IMU...')
            
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
            
            # Set calibration if available
            self.set_calibration()
            
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
            self.set_calibration(calib_data)
        except Exception as e:
            self.get_logger().error(f'Failed to set calibration from parameters: {e}')

    def store_calibration(self):
        """Store current calibration data"""
        try:
            with self.data_lock:
                if not self.connection_active or not self.sensor:
                    self.get_logger().warn('Cannot store calibration: IMU not connected')
                    return
                
                calib_data = self.get_calibration()
            
            calib_file = self.get_parameter('calibFileLoc').get_parameter_value().string_value
            
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(calib_file), exist_ok=True)
            
            data = {
                f'{self.robot_id}_imu': {
                    'ros__parameters': {
                        'calibrateIMU': 1, 
                        'calibOffsetsRadii': calib_data,
                        'calibFileLoc': calib_file
                    }
                }
            }

            with open(calib_file, 'w') as yaml_file:
                yaml.dump(data, yaml_file, default_flow_style=None)
                
            self.get_logger().info(f'Calibration stored to {calib_file}')
             
        except Exception as e:
            self.get_logger().error(f'Failed to store calibration: {e}')

    def set_calibration(self, calib_data=None):
        """Load and apply calibration data"""
        try:
            with self.data_lock:
                if not self.connection_active or not self.sensor:
                    self.get_logger().warn('Cannot set calibration: IMU not connected')
                    return
                
                if calib_data is None:
                    # Load from file
                    calib_file = self.get_parameter('calibFileLoc').get_parameter_value().string_value
                    
                    if not os.path.exists(calib_file):
                        self.get_logger().info('No calibration file found, using defaults')
                        return
                    
                    try:
                        with open(calib_file, 'r') as f:
                            data = yaml.safe_load(f)
                        
                        # Extract calibration data from YAML structure
                        robot_data = data.get(f'{self.robot_id}_imu', {})
                        params = robot_data.get('ros__parameters', {})
                        calib_data = params.get('calibOffsetsRadii', [])
                        
                    except Exception as e:
                        self.get_logger().warn(f'Failed to load calibration file: {e}')
                        return
                
                if len(calib_data) != 22:
                    self.get_logger().warn(f'Invalid calibration data length: {len(calib_data)}, expected 22')
                    return
                
                # Apply calibration
                self.apply_calibration_data(calib_data)
                self.get_logger().info('Calibration applied successfully')
                
        except Exception as e:
            self.get_logger().error(f'Set calibration error: {e}')

    def apply_calibration_data(self, calib_data):
        """Apply calibration data to sensor"""
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

    def get_calibration(self):
        """Get current calibration data from sensor"""
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

    def timer_callback(self):
        """Main IMU reading callback with error handling"""
        try:
            with self.data_lock:
                if not self.connection_active or not self.sensor:
                    # Don't publish anything if IMU is not connected
                    return
                
                success = self.read_and_publish_imu_data()
                
                if success:
                    self.last_successful_read = time.time()
                    
        except Exception as e:
            self.get_logger().error(f'IMU timer callback error: {e}')

    def read_and_publish_imu_data(self):
        """Read and publish IMU data"""
        try:
            # Read quaternion data
            quaternion_data = self.sensor.quaternion
            if quaternion_data and all(q is not None for q in quaternion_data):
                msg_quat = Quaternion()
                msg_quat.x, msg_quat.y, msg_quat.z, msg_quat.w = quaternion_data
                msg_quat.x = float(msg_quat.x)
                msg_quat.y = float(msg_quat.y)
                msg_quat.z = float(msg_quat.z)
                msg_quat.w = float(msg_quat.w)
                self.publisher_quaternion.publish(msg_quat)

            # Read Euler angle data
            euler_data = self.sensor.euler
            if euler_data and all(e is not None for e in euler_data):
                msg_euler = Float32MultiArray()
                msg_euler.data = [float(euler_data[0]), float(euler_data[1]), float(euler_data[2])]
                # Apply heading offset
                msg_euler.data[0] = (msg_euler.data[0] + self.heading_offset) % 360
                self.publisher_euler.publish(msg_euler)

            # Read calibration status
            calib_status = self.sensor.calibration_status
            if calib_status:
                msg_calib = Int16MultiArray()
                msg_calib.data = [int(status) for status in calib_status]
                self.publisher_calib.publish(msg_calib)

            return True
            
        except Exception as e:
            self.get_logger().error(f'IMU data read error: {e}')
            return False

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