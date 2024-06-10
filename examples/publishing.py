# Copyright 2024 iCampus Wildau.
#
# Licensed under the Apache License, Version 2.0.
# See LICENSE in the project root for license information.

"""
This script demonstrates how to publish i2c messages to the ros-i2c package.
"""

# MD+flag:IGNORE:START
from rclpy.node import Node

node = Node("i2c_publisher")
# MD+flag:IGNORE:END


# Import the i2c interfaces
from ros_i2c_interfaces.msg import WriteArray
from ros_i2c_interfaces.msg import WriteByte
from ros_i2c_interfaces.msg import WriteWord

# Create publishers for the i2c messages
pub_i2c_array = node.create_publisher(WriteArray, "i2c/write_array", 10)
pub_i2c_byte = node.create_publisher(WriteByte, "i2c/write_byte", 10)
pub_i2c_word = node.create_publisher(WriteWord, "i2c/write_word", 10)


# Methods to publish i2c messages
def publish_array(self, cmd: int, data: list[int]):
    o = WriteArray()
    o.address = self.i2c_address
    o.command = cmd
    o.data = data

    pub_i2c_array.publish(o)


def publish_byte(self, cmd: int, data: int):
    o = WriteByte()
    o.address = self.i2c_address
    o.command = cmd
    o.data = data

    pub_i2c_byte.publish(o)


def publish_word(self, cmd: int, data: int):
    o = WriteWord()
    o.address = self.i2c_address
    o.command = cmd
    o.data = data

    pub_i2c_word.publish(o)
