# cmd_roboteq.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import BatteryState

import serial
import os
import threading
import time
import re

class MotorDriver(Node):
    def __init__(self, serial_port='/dev/ttyACM0', baudrate=115200):
        # Get robot ID from parameter or environment
        robot_id = os.getenv("ROBOT_ID", "pX")
        super().__init__(f'{robot_id}_cmd_roboteq')
        
        # Declare parameters with defaults from system_config.yaml structure
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', robot_id),
                ('serial_port', serial_port),
                ('baudrate', baudrate),
                ('battery_monitoring.enabled', True),
                ('battery_monitoring.update_interval', 1.0)
            ]
        )
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # Battery monitoring parameters
        self.battery_enabled = self.get_parameter('battery_monitoring.enabled').get_parameter_value().bool_value
        self.battery_interval = self.get_parameter('battery_monitoring.update_interval').get_parameter_value().double_value
        
        # Battery monitoring state
        self.last_battery_voltage = 0.0
        self.battery_lock = threading.Lock()
        
        # Log loaded configuration
        self.get_logger().info(f'Motor Driver Configuration:')
        self.get_logger().info(f'  Robot ID: {self.robot_id}')
        self.get_logger().info(f'  Serial Port: {serial_port}')
        self.get_logger().info(f'  Baudrate: {baudrate}')
        self.get_logger().info(f'  Battery Monitoring: {self.battery_enabled}')
        if self.battery_enabled:
            self.get_logger().info(f'  Battery Update Interval: {self.battery_interval}s')

        try:
            self.roboteq_obj = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            self.get_logger().info(f'Successfully connected to motor controller on {serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to motor controller: {e}')
            raise

        self.subscription = self.create_subscription(
            Int32MultiArray,
            f'/{self.robot_id}/ch_vals',
            self.cmd_callback,
            5)
        self.subscription  # prevent unused variable warning
        
        # Battery state publisher
        if self.battery_enabled:
            self.battery_publisher = self.create_publisher(
                BatteryState,
                f'/{self.robot_id}/battery_state',
                5)
            
            # Battery monitoring timer
            self.battery_timer = self.create_timer(
                self.battery_interval,
                self.battery_monitoring_callback)
            
            self.get_logger().info(f'Battery monitoring enabled on topic /{self.robot_id}/battery_state')

    def move_motor_ch1(self, val):
        try:
            payload1 = f"!G 1 {val}_"
            self.roboteq_obj.write(str.encode(payload1))
        except serial.SerialException as e:
            self.get_logger().error(f'Error writing to motor ch1: {e}')
    
    def move_motor_ch2(self, val):
        try:
            payload2 = f"!G 2 {-val}_"
            self.roboteq_obj.write(str.encode(payload2))
        except serial.SerialException as e:
            self.get_logger().error(f'Error writing to motor ch2: {e}')
            
    def cmd_callback(self, msg):
        if len(msg.data) >= 2:
            inCmd1 = msg.data[0]
            inCmd2 = msg.data[1]
            self.move_motor_ch1(inCmd1)
            self.move_motor_ch2(inCmd2)
        else:
            self.get_logger().warn('Invalid motor command received')

    def read_battery_voltage(self):
        """Read battery voltage from Roboteq controller using ?V command"""
        try:
            # Send voltage query command
            query_cmd = "?V_"
            self.roboteq_obj.write(str.encode(query_cmd))
            
            # Read response with timeout
            response = self.roboteq_obj.readline().decode('utf-8').strip()
            
            # Parse response (expected format: "V=135:246:4730")
            if response:
                # Remove any trailing underscore and parse
                response = response.replace('_', '')
                if '=' in response:
                    voltage_data = response.split('=')[1]
                    
                    # Split by colon to get multiple voltage values
                    voltage_parts = voltage_data.split(':')
                    if len(voltage_parts) >= 2:
                        # Use second value as battery voltage (index 1)
                        # Convert from centi-volts to volts (246 -> 24.6V)
                        battery_voltage = float(voltage_parts[1]) / 10.0
                        
                        self.get_logger().debug(f'Raw voltage response: {response}')
                        self.get_logger().debug(f'Internal: {float(voltage_parts[0])/10.0:.1f}V, Battery: {battery_voltage:.1f}V, 5V: {float(voltage_parts[2])/10.0:.1f}V' if len(voltage_parts) >= 3 else f'Voltages: {voltage_parts}')
                        
                        return battery_voltage
                    else:
                        self.get_logger().debug(f'Insufficient voltage values in response: {response}')
                        return None
                else:
                    self.get_logger().debug(f'Invalid response format: {response}')
                    return None
            return None
            
        except (serial.SerialException, ValueError, IndexError) as e:
            self.get_logger().debug(f'Error reading battery voltage: {e}')
            return None

    def battery_monitoring_callback(self):
        """Timer callback for battery voltage monitoring"""
        if not self.battery_enabled:
            return
            
        try:
            with self.battery_lock:
                voltage = self.read_battery_voltage()
                
                if voltage is not None:
                    self.last_battery_voltage = voltage
                    
                    # Create and publish BatteryState message
                    battery_msg = BatteryState()
                    battery_msg.header.stamp = self.get_clock().now().to_msg()
                    battery_msg.header.frame_id = f"{self.robot_id}_battery"
                    
                    # Voltage information
                    battery_msg.voltage = voltage
                    battery_msg.present = True
                    
                    # Power supply health (always good for simple monitoring)
                    battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
                    battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
                    battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
                    
                    # Unknown values (set to NaN or 0 as appropriate)
                    battery_msg.current = float('nan')
                    battery_msg.charge = float('nan')
                    battery_msg.capacity = float('nan')
                    battery_msg.percentage = float('nan')
                    battery_msg.design_capacity = float('nan')
                    
                    self.battery_publisher.publish(battery_msg)
                    self.get_logger().debug(f'Battery voltage: {voltage:.2f}V')
                else:
                    self.get_logger().debug('Failed to read battery voltage')
                    
        except Exception as e:
            self.get_logger().error(f'Battery monitoring error: {e}')

    def destroy_node(self):
        """Clean up serial connection when destroying node"""
        try:
            if hasattr(self, 'roboteq_obj') and self.roboteq_obj.is_open:
                self.roboteq_obj.close()
                self.get_logger().info('Motor controller connection closed')
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        motor_driver = MotorDriver()
        rclpy.spin(motor_driver)
    except Exception as e:
        print(f"Motor driver failed: {e}")
    finally:
        try:
            motor_driver.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()