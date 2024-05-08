# Copyright 2024 iCampus Wildau.
#
# Licensed under the Apache License, Version 2.0.
# See LICENSE in the project root for license information.

"""This script writes a single word (`0x4237`) to the I²C device at address `0x01`
using command `0x02`. It does so by publishing to the `i2c/write_word` topic.
"""

# MD+flag:IGNORE:START

import rclpy

from ros_i2c_interfaces.msg import WriteWord

rclpy.init()

node = rclpy.create_node("write_word_node")

# MD+flag:IGNORE:END


# Create a new publisher to write a single word to an I²C device.
i2c_write_word = node.create_publisher(WriteWord, "i2c/write_word", 10)

message = WriteWord(address=0x01, command=0x2, data=0x4237)

# MD+flag:IGNORE:START
node.get_logger().info(f"i2c/write_word: {message}")
# MD+flag:IGNORE:END

# Publish the message with the data.
i2c_write_word.publish(message)


# MD+flag:IGNORE:START

node.destroy_node()
rclpy.shutdown()

# MD+flag:IGNORE:END
