# cmd_roboteq.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Int32MultiArray

import serial
import os

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
                ('baudrate', baudrate)
            ]
        )
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # Log loaded configuration
        self.get_logger().info(f'Motor Driver Configuration:')
        self.get_logger().info(f'  Robot ID: {self.robot_id}')
        self.get_logger().info(f'  Serial Port: {serial_port}')
        self.get_logger().info(f'  Baudrate: {baudrate}')

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