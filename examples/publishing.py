"""
This script demonstrates how to publish i2c messages to the ros-i2c package.
"""

# MD+flag:IGNORE:START
from rclpy.node import Node
node = Node("i2c_publisher")
# MD+flag:IGNORE:END


# Import the i2c interfaces
from ros_i2c_interfaces.msg import Write8, Write16, WriteArray

# Create publishers for the i2c messages
pub_i2c_8 = node.create_publisher(Write8, "i2c/write8", 10)
pub_i2c_16 = node.create_publisher(Write16, "i2c/write16", 10) 
pub_i2c_array = node.create_publisher(WriteArray, "i2c/write_array", 10)

# Methods to publish i2c messages
def publish_8_bit(self, cmd: int, data: int):
    o = Write8()
    o.address = self.i2c_address
    o.command = cmd
    o.data = data
    
    pub_i2c_8.publish(o)

def publish_16_bit(self, cmd: int, data: int):
    o = Write16()
    o.address = self.i2c_address
    o.command = cmd
    o.data = data
    
    pub_i2c_16.publish(o)

def publish_array(self, cmd: int, data: list[int]):
    o = WriteArray()
    o.address = self.i2c_address
    o.command = cmd
    o.data = data
    
    pub_i2c_array.publish(o)
