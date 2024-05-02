"""This script tries to write a single byte (`0x42`) to the I²C device at address
`0x01` using command `0x01`. It does so by using the `i2c/try_write_byte` service.
"""

# MD+flag:IGNORE:START

import sys

import rclpy

from ros_i2c_interfaces.msg import WriteByte
from ros_i2c_interfaces.srv import TryWriteByte

rclpy.init()

node = rclpy.create_node("try_write_byte_node")

# MD+flag:IGNORE:END


# Create a new service client for writing a single byte to an I²C device.
i2c_try_write_byte = node.create_client(TryWriteByte, "i2c/try_write_byte")

message = WriteByte(address=0x01, command=0x1, data=0x42)
request = TryWriteByte.Request(message=message)

# MD+flag:IGNORE:START
node.get_logger().debug(f"i2c/try_write_byte: {request}")
# MD+flag:IGNORE:END

# Call the service with the data/message.
future = i2c_try_write_byte.call_async(request)

# Wait until the service call completes.
rclpy.spin_until_future_complete(node, future)


# MD+flag:IGNORE:START

node.destroy_node()
rclpy.shutdown()

# MD+flag:IGNORE:END


# Use the response.
if future.result().success:
    print("Wrote 0x42 to 0x01 using command 0x01.")

else:
    print("Failed to write to I²C device at 0x01.", file=sys.stderr)

    exit(1)
