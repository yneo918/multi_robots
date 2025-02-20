import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import String
import subprocess

global board_id
board_id = 0

contactorPin = 4
try:
    import lgpio
    h = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_output(h, contactorPin)
    board_id = 1
except:
    pass

try:
    import Jetson.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(contactorPin, GPIO.OUT, initial=GPIO.HIGH)
    board_id = 2
except:
    pass

class EnableService(Node):

    def __init__(self):
        super().__init__('en_rov')
        self.srv = self.create_service(SetBool, '/rov/en', self.enable_pin)

    def enable_pin(self, request, response):
        response.success = request.data

        #led_pin = 7
        #GPIO.setmode(GPIO.BOARD) 
        #GPIO.setup(led_pin, GPIO.OUT, initial=GPIO.LOW)
        
        contactorPin = 4

        if response.success == True:
            if board_id == 1:   #  raspberry pi
                lgpio.gpio_write(h, contactorPin, 0)
            elif board_id == 2:
                GPIO.output(led_pin, GPIO.HIGH) 
        else:
            if board_id == 1:   #  raspberry pi
                lgpio.gpio_write(h, contactorPin, 1)
            elif board_id == 2:
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
