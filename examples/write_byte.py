"""This script writes a single byte (`0x42`) to the I²C device at address `0x01`
using command `0x01`. It does so by publishing to the `i2c/write_byte` topic.
"""

# MD+flag:IGNORE:START

import rclpy

from ros_i2c_interfaces.msg import WriteByte

rclpy.init()

node = rclpy.create_node("write_byte_node")

# MD+flag:IGNORE:END


# Create a new publisher to write a single byte to an I²C device.
i2c_write_byte = node.create_publisher(WriteByte, "i2c/write_byte", 10)

message = WriteByte(address=0x01, command=0x1, data=0x42)


# MD+flag:IGNORE:START
node.get_logger().info(f"i2c/write_byte: {message}")
# MD+flag:IGNORE:END


# Publish the message with the data.
i2c_write_byte.publish(message)


# MD+flag:IGNORE:START

node.destroy_node()
rclpy.shutdown()

# MD+flag:IGNORE:END
