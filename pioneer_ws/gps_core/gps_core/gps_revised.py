import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import NavSatFix

import adafruit_gps
import serial
import os
import datetime
import threading
import time
import traceback


class EnhancedReadGPS(Node):
    def __init__(self):
        self.robot_id = os.getenv("ROBOT_ID")
        super().__init__(f'{self.robot_id}_gps1')
        
        # Thread-safe callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Configuration parameters
        self.device_path = "/dev/ttyUSB0"
        self.baudrate = 9600
        self.timeout = 10
        self.retry_attempts = 5
        self.retry_delay = 2.0
        
        # GPS connection state
        self.uart = None
        self.gps_serial = None
        self.connection_healthy = False
        self.last_successful_read = None
        
        # Data filtering and averaging
        self.moving_avg_r = 0.2
        self.lat_avg = 0.0
        self.lon_avg = 0.0
        self.nodata = True
        
        # Statistics for monitoring
        self.read_attempts = 0
        self.successful_reads = 0
        self.connection_attempts = 0
        self.last_reconnect_time = None
        
        # Publisher
        self.gps_publisher = self.create_publisher(NavSatFix, f'/{self.robot_id}/gps1', 1)
        
        # Initialize GPS connection
        self._initialize_gps_connection()
        
        # Main timer for GPS reading
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(
            timer_period, 
            self.timer_callback,
            callback_group=self.callback_group)
        
        # Health monitoring timer
        self.health_timer = self.create_timer(
            5.0,  # Check every 5 seconds
            self.health_check_callback,
            callback_group=self.callback_group)
        
        # Reconnection timer (activated when needed)
        self.reconnect_timer = None
        
        self.get_logger().info(f'Enhanced GPS node initialized for {self.robot_id}')
    
    def _initialize_gps_connection(self):
        """Initialize GPS connection with retry mechanism"""
        for attempt in range(self.retry_attempts):
            try:
                self.connection_attempts += 1
                self.get_logger().info(f'Attempting GPS connection {attempt + 1}/{self.retry_attempts}')
                
                # Close any existing connection
                self._close_connection()
                
                # Try to establish new connection
                self.uart = serial.Serial(
                    self.device_path, 
                    baudrate=self.baudrate, 
                    timeout=self.timeout)
                
                # Test if the connection is working
                if not self.uart.is_open:
                    raise serial.SerialException("Serial port failed to open")
                
                self.gps_serial = adafruit_gps.GPS(self.uart, debug=False)
                
                # Configure GPS module
                self._configure_gps()
                
                # Test GPS communication
                if self._test_gps_communication():
                    self.connection_healthy = True
                    self.get_logger().info('GPS connection established successfully')
                    return True
                else:
                    raise Exception("GPS communication test failed")
                    
            except Exception as e:
                self.get_logger().warn(f'GPS connection attempt {attempt + 1} failed: {str(e)}')
                self._close_connection()
                
                if attempt < self.retry_attempts - 1:
                    time.sleep(self.retry_delay * (attempt + 1))  # Exponential backoff
        
        self.get_logger().error('Failed to establish GPS connection after all attempts')
        self.connection_healthy = False
        return False
    
    def _configure_gps(self):
        """Configure GPS module settings"""
        try:
            # Turn on the basic GGA and RMC info (what you typically want)
            self.gps_serial.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
            time.sleep(0.1)
            
            # Set update rate to 10Hz for better responsiveness
            self.gps_serial.send_command(b"PMTK220,100")
            time.sleep(0.1)
            
            self.get_logger().info('GPS module configured successfully')
            
        except Exception as e:
            self.get_logger().error(f'GPS configuration failed: {str(e)}')
            raise
    
    def _test_gps_communication(self):
        """Test GPS communication by attempting to read data"""
        try:
            for _ in range(10):  # Try reading for up to 1 second
                self.gps_serial.update()
                time.sleep(0.1)
                
                # If we get any data, consider it successful
                if (hasattr(self.gps_serial, 'has_fix') and 
                    hasattr(self.gps_serial, 'latitude')):
                    return True
            
            return False
            
        except Exception as e:
            self.get_logger().error(f'GPS communication test error: {str(e)}')
            return False
    
    def _close_connection(self):
        """Safely close GPS connection"""
        try:
            if self.uart and self.uart.is_open:
                self.uart.close()
        except Exception as e:
            self.get_logger().warn(f'Error closing GPS connection: {str(e)}')
        finally:
            self.uart = None
            self.gps_serial = None
            self.connection_healthy = False
    
    def _attempt_reconnection(self):
        """Attempt to reconnect to GPS device"""
        if self.last_reconnect_time:
            time_since_last = time.time() - self.last_reconnect_time
            if time_since_last < 30.0:  # Wait at least 30 seconds between reconnection attempts
                return
        
        self.last_reconnect_time = time.time()
        self.get_logger().info('Attempting GPS reconnection...')
        
        # Run reconnection in a separate thread to avoid blocking
        reconnect_thread = threading.Thread(target=self._background_reconnect)
        reconnect_thread.daemon = True
        reconnect_thread.start()
    
    def _background_reconnect(self):
        """Background reconnection process"""
        try:
            success = self._initialize_gps_connection()
            if success:
                self.get_logger().info('GPS reconnection successful')
            else:
                self.get_logger().error('GPS reconnection failed')
                # Schedule another reconnection attempt
                if self.reconnect_timer:
                    self.reconnect_timer.destroy()
                
                self.reconnect_timer = self.create_timer(
                    10.0,  # Try again in 10 seconds
                    self._single_reconnect_attempt,
                    callback_group=self.callback_group)
                
        except Exception as e:
            self.get_logger().error(f'Background reconnection error: {str(e)}')
    
    def _single_reconnect_attempt(self):
        """Single reconnection attempt from timer"""
        if self.reconnect_timer:
            self.reconnect_timer.destroy()
            self.reconnect_timer = None
        
        if not self.connection_healthy:
            self._attempt_reconnection()
    
    def timer_callback(self):
        """Enhanced timer callback with error recovery"""
        msg = NavSatFix()
        
        try:
            # Check if connection is healthy
            if not self.connection_healthy or not self.gps_serial:
                msg.status.status = -1  # No fix due to connection issues
                self.gps_publisher.publish(msg)
                return
            
            self.read_attempts += 1
            
            # Update GPS data
            self.gps_serial.update()
            
            lat = self.gps_serial.latitude
            lon = self.gps_serial.longitude
            status = self.gps_serial.fix_quality
            satellites = self.gps_serial.satellites
            
            # Validate GPS data
            if self._validate_gps_data(lat, lon):
                self.successful_reads += 1
                self.last_successful_read = time.time()
                
                # Apply exponential moving average
                if self.nodata:
                    self.lat_avg = lat
                    self.lon_avg = lon
                    self.nodata = False
                    self.get_logger().info(f'First valid GPS reading: {lat:.6f}, {lon:.6f}')
                else:
                    self.lat_avg = (1 - self.moving_avg_r) * self.lat_avg + self.moving_avg_r * lat
                    self.lon_avg = (1 - self.moving_avg_r) * self.lon_avg + self.moving_avg_r * lon
                
                # Populate message
                msg.latitude = self.lat_avg
                msg.longitude = self.lon_avg
                msg.altitude = self.gps_serial.altitude_m if self.gps_serial.altitude_m else 0.0
                
                msg.status.status = status if status is not None else 0
                msg.status.service = satellites if satellites is not None else 0
                msg.header.frame_id = 'primary_gps'
                msg.header.stamp = self.get_clock().now().to_msg()
                
                # Log occasionally for monitoring
                if self.successful_reads % 100 == 0:  # Every 100 successful reads
                    self.get_logger().info(
                        f'GPS reading #{self.successful_reads}: '
                        f'lat={self.lat_avg:.6f}, lon={self.lon_avg:.6f}, '
                        f'status={status}, satellites={satellites}')
            
            else:
                # Invalid GPS data
                msg.status.status = 0  # No fix
                msg.header.frame_id = 'primary_gps'
                msg.header.stamp = self.get_clock().now().to_msg()
                
        except serial.SerialException as e:
            self.get_logger().error(f'GPS serial error: {str(e)}')
            self.connection_healthy = False
            msg.status.status = -1
            self._attempt_reconnection()
            
        except Exception as e:
            self.get_logger().error(f'GPS timer callback error: {str(e)}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            msg.status.status = -1
        
        try:
            self.gps_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish GPS message: {str(e)}')
    
    def _validate_gps_data(self, lat, lon):
        """Validate GPS data quality"""
        if lat is None or lon is None:
            return False
        
        # Check for reasonable coordinate ranges
        if abs(lat) > 90 or abs(lon) > 180:
            self.get_logger().warn(f'GPS coordinates out of range: {lat}, {lon}')
            return False
        
        # Check for zero coordinates (common when no fix)
        if lat == 0.0 and lon == 0.0:
            return False
        
        return True
    
    def health_check_callback(self):
        """Monitor GPS health and performance"""
        try:
            current_time = time.time()
            
            # Calculate success rate
            if self.read_attempts > 0:
                success_rate = (self.successful_reads / self.read_attempts) * 100
            else:
                success_rate = 0
            
            # Check if we haven't had a successful read recently
            if self.last_successful_read:
                time_since_last_read = current_time - self.last_successful_read
                if time_since_last_read > 30.0:  # No successful read for 30 seconds
                    self.get_logger().warn(
                        f'No successful GPS reading for {time_since_last_read:.1f} seconds')
                    self.connection_healthy = False
                    self._attempt_reconnection()
            
            # Log health statistics periodically
            if hasattr(self, '_last_health_log'):
                if current_time - self._last_health_log > 60.0:  # Every minute
                    self._log_health_statistics(success_rate)
                    self._last_health_log = current_time
            else:
                self._last_health_log = current_time
            
            # Check connection health
            if not self.connection_healthy and not self.reconnect_timer:
                self._attempt_reconnection()
                
        except Exception as e:
            self.get_logger().error(f'Health check error: {str(e)}')
    
    def _log_health_statistics(self, success_rate):
        """Log health statistics"""
        self.get_logger().info(
            f'GPS Health Report: '
            f'Success Rate: {success_rate:.1f}% '
            f'({self.successful_reads}/{self.read_attempts}), '
            f'Connection Attempts: {self.connection_attempts}, '
            f'Healthy: {self.connection_healthy}')


def main(args=None):
    """Enhanced main function with proper error handling"""
    try:
        rclpy.init(args=args)
        
        # Use MultiThreadedExecutor for better concurrent handling
        executor = MultiThreadedExecutor()
        
        gps_node = EnhancedReadGPS()
        executor.add_node(gps_node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            gps_node.get_logger().info('Keyboard interrupt received, shutting down...')
        except Exception as e:
            gps_node.get_logger().error(f'Executor error: {str(e)}')
        finally:
            gps_node._close_connection()
            gps_node.destroy_node()
            
    except Exception as e:
        print(f'Failed to initialize GPS node: {str(e)}')
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()