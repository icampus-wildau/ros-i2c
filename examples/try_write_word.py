"""This script tries to write a single word (`0x4237`) to the I²C device at address
`0x01` using command `0x02`. It does so by using the `i2c/try_write_word` service.
"""

# MD+flag:IGNORE:START

import sys

import rclpy

from ros_i2c_interfaces.msg import WriteWord
from ros_i2c_interfaces.srv import TryWriteWord

rclpy.init()

node = rclpy.create_node("try_write_word_node")

# MD+flag:IGNORE:END


# Create a new service client for writing a single word to an I²C device.
i2c_try_write_word = node.create_client(TryWriteWord, "i2c/try_write_word")

message = WriteWord(address=0x01, command=0x2, data=0x4237)
request = TryWriteWord.Request(message=message)

# MD+flag:IGNORE:START
node.get_logger().debug(f"i2c/try_write_byte: {request}")
# MD+flag:IGNORE:END

# Call the service with the data/message.
future = i2c_try_write_word.call_async(request)

# Wait until the service call completes.
rclpy.spin_until_future_complete(node, future)


# MD+flag:IGNORE:START

node.destroy_node()
rclpy.shutdown()

# MD+flag:IGNORE:END


# Use the response.
if future.result().success:
    print("Wrote 0x4237 to 0x01 using command 0x02.")

else:
    print("Failed to write to I²C device at 0x01.", file=sys.stderr)

    exit(1)
