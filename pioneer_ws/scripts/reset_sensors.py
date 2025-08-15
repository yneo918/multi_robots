#!/usr/bin/env python3

"""
Reset command script for Pioneer ROS system
This script provides a reliable way to send reset commands to GPS and IMU sensors
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import argparse
import time
import sys


class ResetCommandSender(Node):
    def __init__(self, robot_id):
        super().__init__(f'{robot_id}_reset_sender')
        self.robot_id = robot_id
        
        # Create publishers for reset commands
        self.gps_reset_pub = self.create_publisher(
            Bool, 
            f'/{robot_id}/reset_gps', 
            1
        )
        
        self.imu_reset_pub = self.create_publisher(
            Bool, 
            f'/{robot_id}/reset_imu', 
            1
        )
        
        # Wait for publishers to be ready
        time.sleep(1.0)

    def send_gps_reset(self, retries=3):
        """Send GPS reset command with retries"""
        msg = Bool()
        msg.data = True
        
        for attempt in range(retries):
            self.get_logger().info(f'Sending GPS reset command (attempt {attempt + 1}/{retries})')
            
            # Send multiple times to ensure delivery
            for _ in range(3):
                self.gps_reset_pub.publish(msg)
                time.sleep(0.1)
            
            # Wait for acknowledgment
            time.sleep(2.0)
            
            self.get_logger().info(f'GPS reset command sent (attempt {attempt + 1})')
            
            if attempt < retries - 1:
                time.sleep(1.0)  # Wait before retry
        
        return True

    def send_imu_reset(self, retries=3):
        """Send IMU reset command with retries"""
        msg = Bool()
        msg.data = True
        
        for attempt in range(retries):
            self.get_logger().info(f'Sending IMU reset command (attempt {attempt + 1}/{retries})')
            
            # Send multiple times to ensure delivery
            for _ in range(3):
                self.imu_reset_pub.publish(msg)
                time.sleep(0.1)
            
            # Wait for acknowledgment
            time.sleep(2.0)
            
            self.get_logger().info(f'IMU reset command sent (attempt {attempt + 1})')
            
            if attempt < retries - 1:
                time.sleep(1.0)  # Wait before retry
        
        return True

    def send_both_resets(self, retries=3):
        """Send both GPS and IMU reset commands"""
        self.get_logger().info('Sending reset commands for both GPS and IMU')
        
        # Send GPS reset first
        self.send_gps_reset(retries)
        
        # Wait between resets
        time.sleep(1.0)
        
        # Send IMU reset
        self.send_imu_reset(retries)
        
        self.get_logger().info('Both reset commands completed')


def main():
    parser = argparse.ArgumentParser(description='Send reset commands to Pioneer sensors')
    parser.add_argument('--robot-id', type=str, default=None,
                       help='Robot ID (default: from ROBOT_ID environment variable)')
    parser.add_argument('--sensor', choices=['gps', 'imu', 'both'], default='both',
                       help='Which sensor to reset (default: both)')
    parser.add_argument('--retries', type=int, default=3,
                       help='Number of retry attempts (default: 3)')
    parser.add_argument('--interactive', action='store_true',
                       help='Interactive mode - ask for confirmation')
    
    args = parser.parse_args()
    
    # Get robot ID
    robot_id = args.robot_id
    if not robot_id:
        import os
        robot_id = os.getenv('ROBOT_ID')
        if not robot_id:
            print("Error: Robot ID not specified and ROBOT_ID environment variable not set")
            print("Use --robot-id argument or set ROBOT_ID environment variable")
            sys.exit(1)
    
    print(f"Reset command script for robot: {robot_id}")
    print(f"Target sensor(s): {args.sensor}")
    print(f"Retry attempts: {args.retries}")
    
    if args.interactive:
        response = input(f"\nSend reset command(s) to {args.sensor}? (y/N): ")
        if response.lower() != 'y':
            print("Cancelled")
            sys.exit(0)
    
    # Initialize ROS
    rclpy.init()
    
    try:
        # Create reset sender node
        reset_sender = ResetCommandSender(robot_id)
        
        # Send appropriate reset commands
        if args.sensor == 'gps':
            reset_sender.send_gps_reset(args.retries)
        elif args.sensor == 'imu':
            reset_sender.send_imu_reset(args.retries)
        elif args.sensor == 'both':
            reset_sender.send_both_resets(args.retries)
        
        print(f"\nReset command(s) sent successfully to {args.sensor}")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        try:
            reset_sender.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()