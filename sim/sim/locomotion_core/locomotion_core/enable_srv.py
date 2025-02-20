import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import String
import subprocess

import time
import Jetson.GPIO as GPIO

class EnableService(Node):

    def __init__(self):
        super().__init__('en_rov')
        self.srv = self.create_service(SetBool, '/rov/en', self.enable_pin)

    def enable_pin(self, request, response):
        response.success = request.data
        led_pin = 7
        GPIO.setmode(GPIO.BOARD) 
        GPIO.setup(led_pin, GPIO.OUT, initial=GPIO.LOW)
        if response.success == True:
            GPIO.output(led_pin, GPIO.HIGH) 
        else:
            GPIO.output(led_pin, GPIO.LOW)
        #self.get_logger().info('Incoming request\n\tSet pin to:' + str(response.success))
        
        return response


def main(args=None):
    rclpy.init(args=args)

    en_rov = EnableService()

    rclpy.spin(en_rov)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()