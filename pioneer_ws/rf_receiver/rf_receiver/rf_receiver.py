import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16

from digi.xbee.devices import XBeeDevice, XBee64BitAddress
from typing import Any, Union, Callable, Optional, Tuple

from time import time
from random import randrange


from adaptive_navigation_utilities.pubsub import PubSubManager
from adaptive_navigation_utilities.namespace_builder import HardwareNamespace



# Global variables
Numeric = Union[int, float]
DEBUG: bool = True


class RFReceiver(Node):
    """  
    Creates the RF Receiver node and publish for the RSSI signal.

    Implemented using in ROS2 and Digi-Xbee library.
    """

    PORT: str = 'port'
    BAUD_RATE: str = 'baud_rate'
    FREQ: str = 'freq'
    PUB_TOPIC: str = 'pub_topic'
    RANGE_FAKE_RSSI: Tuple[int] = (-100, -10)


    def __init__(self, func: Callable = None):

        # Create node
        # Note: We use the __name__ macro and
        # parse for the file name to use as
        # the node name
        super().__init__(__name__.split('.')[-1])
    
        # Get namespace
        self.ns: HardwareNamespace = HardwareNamespace(self.get_namespace())
    
        # Declare parameters provided with default params
        # if not using ros2 launch
        self.declare_parameters(
            namespace='',
            parameters=[
                (RFReceiver.PORT, "/dev/ttyUSB1"),
                (RFReceiver.BAUD_RATE, 115200),
                (RFReceiver.FREQ, 2),
                (RFReceiver.PUB_TOPIC, 'rssi')
            ]
        )

        # Create PubSubManager
        self.pubsub: PubSubManager = PubSubManager(self)


        # Set attributes from parameters
        # Note: Parameters are at the node level
        #       while attributes are the class level
        self.port: str = self.get(RFReceiver.PORT)
        self.baud_rate: str = self.get(RFReceiver.BAUD_RATE)
        self.freq: int = self.get(RFReceiver.FREQ)

        # Create attributes based on messages
        # TODO: Make a data structure for this?
        self.msg: str = ''
        self.remote_device: XBee64BitAddress = None
        self.timestamp: Numeric  = None
        self.rssi: int = None

        # Add aliases for logging
        self.debug: Callable = self.log().debug
        self.info: Callable = self.log().info
        self.warn: Callable = self.log().warn
        self.error: Callable = self.log().error
    
        self.info(f"Namespace: {self.ns.namespace}")
        
        # Check if sim
        self.info(f"Is in sim mode: {self.ns.is_simulation}")

        # Create publish topics
        self.create_publish_topics()

        if DEBUG: print(self.port)

        # If not simulation
        if not self.ns.is_simulation:

            # Initialize XBee Device
            self.device: XBeeDevice = self.xbee_init()

            # Open the device
            self.device.open()
            if DEBUG: print("Device is opening...")

            # Add attribute for callback
            if func is None:
                self.callback_func: Callable = \
                    self.get_rssi_from_received_msg
            else:
                self.callback_func: Callable = func

            # Add callback to msg
            self.add_data_received_callback(self.callback_func)

            # Add poll timer
            self.create_timer(self.period, self.wait)

        else:

            # Add poll timer
            self.create_timer(self.period, 
                              self.get_fake_rssi_from_received_msg)
        



    @property
    def period(self) -> Numeric:
        return 1/self.freq

    # Create publish topics
    def create_publish_topics(self) -> None:

        # Create publish topic of message
        self.pubsub.create_publisher(
            Int16,
            self.get(RFReceiver.PUB_TOPIC),
            10
        )

    # Crete alias function for common ROS functions
    def get(self, param) -> Any:
        return self.get_parameter(param).value
    
    def log(self):
        return self.get_logger()

    # Create XBee device
    def xbee_init(self) -> XBeeDevice:
        return XBeeDevice(self.port, self.baud_rate)
    
    # Add callback
    def add_data_received_callback(self, func: Callable) -> None:
        self.device.add_data_received_callback(func)
    
    def update_rssi(self) -> Optional[int]:
        rssi_bytes: bytes = self.device.get_parameter("DB")
        if rssi_bytes:
            return - int.from_bytes(rssi_bytes)
        else:
            return None
    
    def get_rssi_from_received_msg(self, message) -> None:

        # Extract data from received message
        self.msg = message.data.decode()
        self.remote_device = \
                    message.remote_device.get_64bit_addr()
        self.timestamp = message.timestamp

        # Log messages at INFO level
        self.info(f"Received: {self.msg}")
        self.info(f"From: {self.remote_device}")
        self.info(f"Timestamp: {self.timestamp}")


        try:

            # If the device is not already open, open it
            if not self.device.is_open():
                self.device.open()

            # Get rssi
            self.rssi = self.update_rssi()
            self.info(f"RSSI of last message: {self.rssi}")
            
            # Publish data
            self.pubsub.publish(
                self.get(RFReceiver.PUB_TOPIC),
                Int16(data=self.rssi)
            )

        except Exception as e:
            self.error(f"Error receiving RSSI: {e}")

        finally:

            # Close device if caught exception
            if self.device is not None and self.device.is_open():
                self.device.close()
            
    def get_fake_rssi_from_received_msg(self) -> None:

        # Update fake rssi
        self.rssi = randrange(
                    RFReceiver.RANGE_FAKE_RSSI[0],
                    RFReceiver.RANGE_FAKE_RSSI[1]
                        )
        
        # Log rssi
        self.info(f"RSSI: {self.rssi}")

        # Publish fake data
        self.pubsub.publish(
            self.get(RFReceiver.PUB_TOPIC),
            Int16(data=self.rssi)
        )


    def wait(self) -> None:
        if not self.device.is_open():
            self.device.open()
        self.debug(f"Heartbeat at {time()}")
    
    def poll_device(self) -> None:
        if not self.device.is_open():
            self.device.open()
        try:
            self.rssi = self.update_rssi()
            self.info(f"Polled RSSI: {self.rssi}")
        except Exception as e:
            self.error(f"Polling error: {e}")

            
def main(args=None):

    if DEBUG: print(__name__)
    
    # Initialize ros client library
    rclpy.init(args=args)

    # Creates a rf source node
    rf_receiver = RFReceiver()

    try:
        while rclpy.ok():
            rclpy.spin(rf_receiver)
    except KeyboardInterrupt:
        pass
    finally:
        rf_receiver.destroy_node()
        rclpy.shutdown()
        

if __name__ == "__main__":
    main()
