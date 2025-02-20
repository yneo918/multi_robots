import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Bool

global board_id
board_id = 0

contactorPin = 7
# open the gpio chip and set the LED pin as output
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
    board_id = 1
except:
    pass

class duEnable(Node):

    def __init__(self):
        super().__init__('rover2_enable')
        self.inCmd = 0.0
        self.subscription = self.create_subscription(
            Bool,
            'r2/enable',
            self.enable_callback,
            5)
        self.subscription  # prevent unused variable warning

    def contactor_ctrl(self, val):
        if board_id == 1:   #  raspberry pi
            lgpio.gpio_write(h, contactorPin, val)
        elif board_id == 2: # jetson
            if (val == True):
                GPIO.output(contactorPin, GPIO.HIGH)
            else:
                GPIO.output(contactorPin, GPIO.LOW)
        else:
            print("Flag disabled.")

    def enable_callback(self, msg):
        inCmd = msg.data
        self.contactor_ctrl(inCmd)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = duEnable()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
