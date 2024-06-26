# Copyright 2024 iCampus Wildau.
#
# Licensed under the Apache License, Version 2.0.
# See LICENSE in the project root for license information.

"""This script tries to write `"Hello, World!"` to the I²C device at address `0x01`
using command `0x80`. It does so by using the `i2c/try_write_array` service.
"""

# MD+flag:IGNORE:START

import sys

import rclpy

from ros_i2c_interfaces.msg import WriteArray
from ros_i2c_interfaces.srv import TryWriteArray

rclpy.init()

node = rclpy.create_node("try_write_hello_word_node")

# MD+flag:IGNORE:END


# Create a new service client for writing multiple bytes to an I²C device.
i2c_try_write_array = node.create_client(TryWriteArray, "i2c/try_write_array")

data = "Hello, World!".encode("ascii")  # The data to write to the device.
message = WriteArray(address=0x01, command=0x80, data=data, write_length=False)
request = TryWriteArray.Request(message=message)

# MD+flag:IGNORE:START
node.get_logger().debug(f"i2c/try_write_array: {request}")
# MD+flag:IGNORE:END

# Call the service with the data/message.
future = i2c_try_write_array.call_async(request)

# Wait until the service call completes.
rclpy.spin_until_future_complete(node, future)


# MD+flag:IGNORE:START

node.destroy_node()
rclpy.shutdown()

# MD+flag:IGNORE:END


# Use the response.
if future.result().success:
    print("Wrote 'Hello, World!' to 0x01 using command 0x80.")

else:
    print("Failed to write to I²C device at 0x01.", file=sys.stderr)

    exit(1)
