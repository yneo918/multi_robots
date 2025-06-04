import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import adafruit_gps
import serial
import os
import datetime
import time
import threading


class ReadGPS(Node):
    def __init__(self):
        self.robot_id = os.getenv("ROBOT_ID")
        super().__init__(f'{self.robot_id}_gps1')
        
        # GPS configuration parameters
        self.device_paths = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]
        self.baudrate = 9600
        self.timeout = 10
        
        # Connection state
        self.uart = None
        self.gps_serial = None
        self.connection_active = False
        
        # Data smoothing
        self.moving_avg_r = 0.2
        self.lat_avg = 0.0
        self.lon_avg = 0.0
        self.nodata = True
        
        # Error handling
        self.max_retries = 5
        self.retry_delay = 2.0
        self.last_successful_read = time.time()
        self.connection_timeout = 10.0  # seconds
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Publisher
        self.gps_publisher = self.create_publisher(NavSatFix, f'/{self.robot_id}/gps1', 1)
        
        # Initialize GPS connection
        self.initialize_gps_connection()
        
        # Timers
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Health monitoring timer
        health_check_period = 5.0  # seconds
        self.health_timer = self.create_timer(health_check_period, self.health_check_callback)

    def initialize_gps_connection(self):
        """Initialize GPS connection with multiple device path attempts"""
        for device_path in self.device_paths:
            if self.connect_to_device(device_path):
                self.get_logger().info(f'GPS connected successfully to {device_path}')
                return
        
        self.get_logger().error('Failed to connect to any GPS device')

    def connect_to_device(self, device_path):
        """Attempt to connect to a specific device path"""
        try:
            # Check if device exists
            if not os.path.exists(device_path):
                return False
                
            self.get_logger().info(f'Attempting to connect to GPS at {device_path}')
            
            # Create serial connection
            uart = serial.Serial(device_path, baudrate=self.baudrate, timeout=self.timeout)
            gps_serial = adafruit_gps.GPS(uart, debug=False)
            
            # Configure GPS
            gps_serial.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
            gps_serial.send_command(b"PMTK220,100")
            
            # Test the connection by trying to read
            time.sleep(1)
            gps_serial.update()
            
            # If we get here without exception, connection is successful
            with self.data_lock:
                self.uart = uart
                self.gps_serial = gps_serial
                self.connection_active = True
                self.last_successful_read = time.time()
            
            return True
            
        except Exception as e:
            self.get_logger().warn(f'Failed to connect to {device_path}: {e}')
            try:
                if 'uart' in locals():
                    uart.close()
            except:
                pass
            return False

    def reconnect_gps(self):
        """Attempt to reconnect GPS"""
        self.get_logger().warn('Attempting to reconnect GPS...')
        
        # Close existing connection
        try:
            if self.uart:
                self.uart.close()
        except:
            pass
        
        with self.data_lock:
            self.uart = None
            self.gps_serial = None
            self.connection_active = False
        
        # Try to reconnect
        self.initialize_gps_connection()

    def health_check_callback(self):
        """Monitor GPS connection health"""
        current_time = time.time()
        
        if current_time - self.last_successful_read > self.connection_timeout:
            self.get_logger().warn('GPS connection timeout detected')
            with self.data_lock:
                self.connection_active = False
            
            # Attempt reconnection in a separate thread
            threading.Thread(target=self.reconnect_gps, daemon=True).start()

    def timer_callback(self):
        """Main GPS reading callback with robust error handling"""
        msg = NavSatFix()
        
        try:
            with self.data_lock:
                if not self.connection_active or not self.gps_serial:
                    # Publish empty message with status 0
                    msg.status.status = 0
                    self.gps_publisher.publish(msg)
                    return
                
                # Attempt to read GPS data
                success = self.read_gps_data(msg)
                
                if success:
                    self.last_successful_read = time.time()
                
        except Exception as e:
            self.get_logger().error(f'GPS timer callback error: {e}')
            msg.status.status = 0
        
        # Always publish a message
        try:
            self.gps_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish GPS message: {e}')

    def read_gps_data(self, msg):
        """Read GPS data with error handling"""
        try:
            self.gps_serial.update()
            
            lat = self.gps_serial.latitude
            lon = self.gps_serial.longitude
            status = self.gps_serial.fix_quality
            satellites = self.gps_serial.satellites
            
            # Log raw data for debugging
            self.get_logger().debug(f'Raw GPS: lat={lat}, lon={lon}, status={status}, sats={satellites}')
            
            if lat is not None and lon is not None:
                # Apply moving average filter
                if self.nodata:
                    self.lat_avg = lat
                    self.lon_avg = lon
                    self.nodata = False
                else:
                    self.lat_avg = (1 - self.moving_avg_r) * self.lat_avg + self.moving_avg_r * lat
                    self.lon_avg = (1 - self.moving_avg_r) * self.lon_avg + self.moving_avg_r * lon
                
                # Populate message
                msg.latitude = self.lat_avg
                msg.longitude = self.lon_avg
                
                if self.gps_serial.altitude_m is not None:
                    msg.altitude = self.gps_serial.altitude_m
                
                msg.status.status = status if status is not None else 0
                msg.status.service = satellites if satellites is not None else 0
                msg.header.frame_id = 'primary_gps'
                msg.header.stamp = self.get_clock().now().to_msg()
                
                self.get_logger().debug(
                    f'{datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")} '
                    f'GPS: {msg.latitude:.6f}, {msg.longitude:.6f}, alt: {msg.altitude:.1f}'
                )
                
                return True
            else:
                msg.status.status = 0
                return False
                
        except Exception as e:
            self.get_logger().error(f'GPS read error: {e}')
            msg.status.status = 0
            return False

    def destroy_node(self):
        """Clean up resources when destroying node"""
        try:
            if self.uart:
                self.uart.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    retry_count = 0
    max_retries = 3
    
    while retry_count < max_retries:
        try:
            get_gps_data = ReadGPS()
            rclpy.spin(get_gps_data)
            break  # If we get here, normal shutdown occurred
            
        except Exception as e:
            retry_count += 1
            print(f"GPS node failed (attempt {retry_count}): {e}")
            
            if retry_count < max_retries:
                print(f"Retrying in 5 seconds...")
                time.sleep(5)
            else:
                print("GPS node failed after maximum retries")
                
        finally:
            try:
                get_gps_data.destroy_node()
            except:
                pass
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()