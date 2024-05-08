# Copyright 2024 iCampus Wildau.
#
# Licensed under the Apache License, Version 2.0.
# See LICENSE in the project root for license information.

"""This script writes `"Hello, World!"` to the I²C device at address `0x01` using
command `0x80`. It does so by publishing to the `i2c/write_array` topic.
"""

# MD+flag:IGNORE:START

import rclpy

from ros_i2c_interfaces.msg import WriteArray

rclpy.init()

node = rclpy.create_node("write_hello_word_node")

# MD+flag:IGNORE:END


# Create a new publisher to write multiple bytes to an I²C device.
i2c_write_array = node.create_publisher(WriteArray, "i2c/write_array", 10)

data = "Hello, World!".encode("ascii")  # The data to write to the device.
message = WriteArray(address=0x01, command=0x80, data=data, write_length=False)

# MD+flag:IGNORE:START
node.get_logger().info(f"i2c/write_array: {message}")
# MD+flag:IGNORE:END

# Publish the message with the data.
i2c_write_array.publish(message)


# MD+flag:IGNORE:START

node.destroy_node()
rclpy.shutdown()

# MD+flag:IGNORE:END
