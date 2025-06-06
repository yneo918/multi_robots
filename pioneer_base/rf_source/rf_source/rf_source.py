import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import String, Int16

from digi.xbee.devices import XBeeDevice
from typing import Any, Union, Callable

from functools import wraps

from adaptive_navigation_utilities.pubsub import PubSubManager
from adaptive_navigation_utilities.namespace_builder import HardwareNamespace


# Global variables
Numeric = Union[int, float]
DEBUG: bool = True


class RFSource(Node):
    """  
    Creates the RF Source node and continously
    sends broadcast data using the send_broadcast_data
    method. 

    Implemented using in ROS2 and Digi-Xbee library.
    """

    PORT: str = "port"
    BAUD_RATE: str = "baud_rate"
    FREQ: int = "freq"
    PUB_TOPIC: str = "topicname/msg"
    POWER_PUB_TOPIC: str = "power_level_topic_name"
    POWER_LEVEL: str = "power_level"
    
    def __init__(self) -> None:

        # Create node
        # Note: We use the __name__ macro and
        # parse for the file name to use as
        # the node name
        super().__init__(__name__.split('.')[-1])

        # Get namespace
        self.ns: HardwareNamespace = HardwareNamespace(self.get_namespace())
    
        # Add aliases for logging
        self.info: Callable = self.log().info
        self.warn: Callable = self.log().warn
        self.error: Callable = self.log().error
        self.debug: Callable = self.log().debug

        self.info(f"Namespace: {self.ns.namespace}")

        # Create PubSubManager
        self.pubsub: PubSubManager = PubSubManager(self)

        # Declare parameters provided with default params
        # if not using ros2 launch
        self.declare_parameters(
            namespace='',
            parameters=[
                (RFSource.PORT, "/dev/ttyUSB0"),
                (RFSource.BAUD_RATE, 115200),
                (RFSource.FREQ, 0.5),
                (RFSource.PUB_TOPIC, "message"),
                (RFSource.POWER_LEVEL, 0),
                (RFSource.POWER_PUB_TOPIC, "power_level")
            ]
        )

        # Set attributes from parameters
        # Note: Parameters are at the node level
        #       while attributes are the class level
        self.port: str = self.get(RFSource.PORT)
        self.baud_rate: str = self.get(RFSource.BAUD_RATE)
        self.freq: int = self.get(RFSource.FREQ)
        
        # Check if sim
        self.info(f"Is in sim mode: {self.ns.is_simulation}")

        # Create publish topics
        self.create_publish_topics()

        if DEBUG: print(self.port)



        # Initialize XBee Device
        self.device: XBeeDevice = self.xbee_init()

        # Set power level of device
        self.set_power_level(self.get(RFSource.POWER_LEVEL))

        # Add a callback fcn in ROS per timer
        self.timer = self.create_timer(self.period, self.send_data_broadcast)

        # TODO: Fix the naming
        self.timer2 = self.create_timer(10, self.get_power_level )

        self.add_on_set_parameters_callback(self.parameters_callback)

    # Add callback if there is any change in ROS parameters
    # TODO: Fix hardcoding
    def parameters_callback(self, params):

        self.info("Performing parameters callback update")
        for p in params:
            if p.name == RFSource.POWER_LEVEL:

                self.info(f"Updating {RFSource.POWER_LEVEL} with {p.value}")

                # Update power level
                self.set_power_level(p.value)

            if p.name == RFSource.PORT:
                self.info(f"Changing port to {p.value}")
                self.port = p.value

        return SetParametersResult(successful=True)

    def non_simulation(func: Callable):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            if self.ns.is_simulation:
                self.error(f"Cannot run {func.__name__} at while in sim!")
                return
            else:
                result = func(self, *args, **kwargs)
                return result
        return wrapper
    
    def alternate_sim_func(alt_func: Callable):
        @wraps(alt_func)
        def decorator(func: Callable):
            def wrapper(self, *args, **kwargs):
                if self.ns.is_simulation:
                    self.warn(f"Cannot run {func.__name__} at while in sim!")
                    result = alt_func(self, *args, **kwargs)
                else:
                    result = func(self, *args, **kwargs)
                    return result
            return wrapper
        return decorator

    # @property
    # def pl(self) -> int:
        
    #     # TODO: Reduce if using either:
    #     # self.pl vs self.get_power_level() vs self.get()
    #     return self.get(RFSource.POWER_LEVEL)
    
    # @pl.setter
    # def pl(self, val) -> int:
        
    #     # Check if correct power level
    #     self.qualify_power_level(val)

    #     # Set power level
    #     print("hi")
    #     self.set_power_level(val)

    @property
    def period(self) -> Numeric:
        return 1/self.freq
    
    # Create publish topics
    def create_publish_topics(self) -> None:

        # Create publish topic of message
        self.pubsub.create_publisher(
            String,
            self.get(RFSource.PUB_TOPIC),
            10
        )

        # Create publish topic of power level
        self.pubsub.create_publisher(
            Int16,
            self.get(RFSource.POWER_PUB_TOPIC),
            10
        )

    # # TODO: Consider alternatives to ROS params
    # # TODO: Fix hard-coding
    # def update_power_level(self, params) -> None:
        
    #     for param in params:

    #         if param == RFSource.POWER_PUB_TOPIC and param != self.pl:

    #             # Set param
    #             self.pl = param


    # Create alias function for common ROS functions
    def get(self, param) -> Any:
        return self.get_parameter(param).value
    
    # Create alias function for getting logger
    def log(self):
        return self.get_logger()

    # Create XBee device
    @non_simulation
    def xbee_init(self) -> None:
        self.device: XBeeDevice = XBeeDevice(self.port, self.baud_rate)

        # Open the device
        self.device.open()
        if DEBUG: print("Device is opening...")

        if self.device.is_open():
            print("Hooray!")
            print(self.device)
        
        return self.device
    
    def send_fake_data_broadcast(self, msg: str = 'Hello world!') -> None:

        self.info(f"Sending fake message: {msg}")

        # Publish message
        self.pubsub.publish(
            self.get(RFSource.PUB_TOPIC),
            String(data=msg)
        )
    
    # Send broadcast data
    @alternate_sim_func(send_fake_data_broadcast)
    @non_simulation
    def send_data_broadcast(self, msg: str = 'Hello world!') -> None:
        try:

            # If the device is not already open, open it
            if not self.device.is_open():
                self.device.open()

            # Send broadcast message
            self.info(f"Sending message: {msg}")
            self.device.send_data_broadcast(msg)

            # Publish message
            self.pubsub.publish(
                self.get(RFSource.PUB_TOPIC),
                String(date=msg)
            )

        except Exception as e:

            # Print error
            print(f"Error: {e}")
            return False
        
        finally:

            # Close device if caught exception
            if self.device is not None and self.device.is_open():
                self.device.close()
    
    def set_fake_power_level(self, val):

        if hasattr(self, "pl"):
            self.info(f"Current power level {self.pl}")

        # Check if invalid power level
        if self.qualify_power_level(val):

            self.info(f"Setting fake power val to {val}")
            self.pl = val
   
    def qualify_power_level(self, val):

        self.info(f"Power level to be qualified: {val}")

        if val not in [0, 1, 2, 3, 4]:
            self.error("Incorrect power level. Must be from 0 to 4")
            return False
        else:
            if hasattr(self, "pl"):
                self.info(f"Returning {self.pl} as power level")
            return True

    def get_fake_power_level(self):
        if hasattr(self, "pl"):
            self.info(f"Fake power val: {self.pl}")
 

    # This cannot be a property because
    # it actively performs an action 
    # and retrieves a value
    @alternate_sim_func(get_fake_power_level)
    @non_simulation
    def get_power_level(self):

        # If the device is not already open, open it
        if not self.device.is_open():
                self.device.open()

        self.info("Accessing power level!")

        # Get power level
        pl_raw = self.device.get_parameter("PL")

        if pl_raw is None:
            self.error("Failed to read power level.")

            self.pl = None

            return None
        
        else:
            self.pl = int.from_bytes(pl_raw, "big")
            self.info(f"Current power level {self.pl}")
        
        # Close device if caught exception
        if self.device is not None and self.device.is_open():
            self.device.close()

        return


    @alternate_sim_func(set_fake_power_level)
    @non_simulation
    def set_power_level(self, level) -> bool:

        # Check if invalid power level
        if not self.qualify_power_level(level):
            return

        # If the device is not already open, open it
        if not self.device.is_open():
            self.device.open()

        try:
            level = int(level)  # Convert to int if it’s a string like "3"
        except ValueError:
            print(f"Invalid input: level must be an integer, got {level}")
            return False

        try:
            self.device.set_parameter("PL", bytes([level]))
            self.info(f"Set power level to {level}")
            self.pl = level
            return True
        except Exception as e:
            self.error(f"Failed to set power level: {e}")
            return False

        
        

def main(args=None):

    if DEBUG: print(__name__)
    
    # Initialize ros client library
    rclpy.init(args=args)

    # Creates a rf source node
    rf_source = RFSource()

    # Continuously execute the node
    # The execution lies in the RFSource.timer() 
    # function that uses a callback function
    rclpy.spin(rf_source)


if __name__ == '__main__':
    main()