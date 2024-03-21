#!/usr/bin/env python

"""
ROS Node to offer I2C functionality to the rest of the system.

This node subscribes ROS-Messages to send data over I2C.
This node is for the Raspberry Pi 4B.

I2C: 
http://www.netzmafia.de/skripten/hardware/RasPi/RasPi_I2C.html 
and
https://raspberry-projects.com/pi/programming-in-python/i2c-programming-in-python/using-the-i2c-interface-2 

"""

import os
import sys
from typing import Dict

import smbus  # System management bus ==> I2C compatible
import time

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import String
from hardware_interfaces_msgs.msg import I2Cwrite8, I2Cwrite16, I2CwriteArray

from resource_manager.resources import Resources

import threading

sem = threading.Semaphore()


""" 
Create smbus instance and open the instance.

For Raspberry Pi 4:
The Raspberry Pi has two I2C busses, but only bus 1 should be used

I2C Bus 1:
SDA --> Pin 3
SCL --> Pin 5


For Jetson Nano:
The Jetson Nano has two I2C busses.

I2C Bus 0:
SDA --> Pin 27
SCL --> Pin 28

I2C Bus 1:
SDA --> Pin 3
SCL --> Pin 5


If there are any I2C devices attached, you can scan that bus from the command line
$ i2cdetect -y -r 0 
$ i2cdetect -y -r 1
"""


def toUInt8(x):
    return x % 256


class ErrorSlave:
    def __init__(self, address) -> None:
        self.address = address
        self.lastTried = 0

        self.errorCount = 1

    def is_checkable(self):
        # After 10 seconds without error, give it another try
        return time.time() - self.lastTried > 10 or self.errorCount <= 3


class I2CNode(Node):
    def __init__(self, bus=1):
        super().__init__("i2c_bridge_node")

        self.get_logger().info("Try starting I2C Bridge Node")

        rclpy.get_default_context().on_shutdown(self.on_shutdown)

        self.error_slaves: Dict[int, ErrorSlave] = dict()

        # Open I2C Bus
        try:
            self.bus = smbus.SMBus(bus)
            self.get_logger().info("Successfully opened SMbus({})".format(bus))
        except Exception as e:
            self.get_logger().info("{}".format(e))
            self.bus = BusSim(bus, self)

        # Subscribe to I2C Bridge Topics

        self.sub_i2c_8 = self.create_subscription(I2Cwrite8, "system/i2c/write8", self.on_write_8, 10)
        self.sub_i2c_16 = self.create_subscription(I2Cwrite16, "system/i2c/write16", self.on_write_16, 10)
        self.sub_i2c_arr = self.create_subscription(I2CwriteArray, "system/i2c/writeArray", self.on_write_array, 10)

        ################################
        ### CREATE RESOURCE ############

        self.resource_name = self.declare_parameter(
                "resource_name", "hardware_interfaces",
                ParameterDescriptor(description='Resource name of this i2c bridge. Used to wait for the node to be started')
            ).get_parameter_value().string_value
        
        self.i2c_resource_name = self.declare_parameter(
                "i2c_resource_name", f"{self.resource_name}/i2c",
                ParameterDescriptor(description='The i2c resource name.')
            ).get_parameter_value().string_value
        
        self.resource_manager_topic = self.declare_parameter(
                "resource_manager_topic", "resources/manager",
                ParameterDescriptor(description='Resource manager topic name.')
            ).get_parameter_value().string_value
        

        self.resources = Resources(self)
        self.resources.create(self.i2c_resource_name)

        ################################
        ### LOG INFO ###################

        self.get_logger().info("#### SUBSCRIBER ####")
        for s in self.subscriptions:
            self.get_logger().info(f"Subscribed: {s.topic_name:<25} | Msg-Type: {str(s.msg_type)}")

        self.get_logger().info("#### PUBLISHER ####")
        for s in self.publishers:
            self.get_logger().info(f"Publishing: {s.topic_name:<25} | Msg-Type: {str(s.msg_type)}")

        ################################

    def on_shutdown(self):
        self.get_logger().info("#### SHUTDOWN ####")
        self.resources.shutdown()
        self.bus.close()

    def on_write_16(self, msg: I2Cwrite16):
        """
        Send 16 bit data over I2C.

        Parameters
        ----------
        msg : I2Cwrite16
            I2C message with address, command and data.
        """
        self.get_logger().debug("I2C write16:    addr: {} cmd: {} data: {}".format(str(msg.address), str(msg.command), str(msg.data)))
        try:
            self.bus.write_word_data(msg.address, msg.command, msg.data)
        except Exception as e:
            self.get_logger().error(str(e))

    def on_write_8(self, msg: I2Cwrite8):
        """
        Send 8 bit data over I2C.

        Parameters
        ----------
        msg : I2Cwrite8
            I2C message with address, command and data.
        """
        self.get_logger().debug("I2C write8:     addr: {} cmd: {} data: {}".format(str(msg.address), str(msg.command), str(msg.data)))
        try:
            self.bus.write_byte_data(msg.address, msg.command, msg.data)
        except Exception as e:
            self.get_logger().error(str(e))

    def on_write_array(self, msg: I2CwriteArray):
        """
        Send an array of data over I2C.

        Parameters
        ----------
        msg : I2Cwrite16
            I2C message with address, command and an data array.
        """
        data = []

        for d in msg.data:
            data.append(toUInt8(int(d)))

        for i in range(2):
            try:
                if msg.address not in self.error_slaves or self.error_slaves[msg.address].is_checkable():
                    self.get_logger().debug("I2C writeArray: addr: {} cmd: {} data: {}".format(str(msg.address), str(msg.command), data))
                    self.bus.write_block_data(msg.address, msg.command, data)
                    if msg.address in self.error_slaves:
                        self.error_slaves[msg.address].errorCount = 0
                    break
                else:
                    break
            except Exception as e:
                if msg.address in self.error_slaves:
                    self.get_logger().error(f"Updated error slave for {msg.address} because of timeout")
                    self.error_slaves[msg.address].lastTried = time.time()
                    self.error_slaves[msg.address].errorCount += 1
                    break
                else:
                    self.error_slaves[msg.address] = ErrorSlave(msg.address)

                self.get_logger().error(str(e) + " ... Trying again")
                time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)

    node = I2CNode()

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(e)
    except KeyboardInterrupt as e:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


class BusSim(object):
    def __init__(self, num, rosNode):

        self.num = num
        self.node = rosNode
        self.log = self.node.get_logger()

        self.log.warn("####### ERROR OPENING SMbus #######")
        self.log.warn("Starting SMbus Simulator ({})".format(self.num))

    def write_block_data(self, addr, cmd, data):
        self.log.debug("Write @ {} CMD: {} Data: {}".format(addr, cmd, data))

    def write_byte_data(self, addr, cmd, data):
        self.log.debug("Write @ {} CMD: {} Data: {}".format(addr, cmd, data))

    def write_word_data(self, addr, cmd, data):
        self.log.debug("Write @ {} CMD: {} Data: {}".format(addr, cmd, data))


if __name__ == "__main__":
    main()

"""
### Address of the slave device
addr = 0x08

### Example values
cmd = 0x55        # Command or register
byteVal = 0x1A    # byte value
wordVal = 0xABCD  # 2 byte value
listVal = [5, 10, 15, 50]  # value list

### Write operations
bus.write_byte(addr, byteVal)
bus.write_byte_data(addr, cmd, byteVal)
bus.write_word_data(addr, cmd, wordVal)
bus.write_block_data(addr, cmd, listVal)

### Read operations
readByte = bus.read_byte(addr)
readByte = bus.read_byte_data(addr, cmd)
readWord = bus.read_word_data(addr, cmd)
readList = bus.read_block_data(addr, cmd)
"""

""" List of smbus commands

Send only the read / write bit 
long write_quick(int addr)

Read a single byte from a device, without specifying a device register. 
long read_byte(int addr)

Send a single byte to a device (without command or register)
long write_byte(int addr, char val)

Read a single byte from a device (cmd is the command or register declaration)
long read_byte_data(int addr, char cmd)

Send a single byte to a device (cmd is the command or register declaration)
long write_byte_data(int addr, char cmd, char val)

Read a 16 Bit word from a device (cmd is the command or register declaration)
long read_word_data(int addr, char cmd)

Send a 16 Bit word from to a device (cmd is the command or register declaration)
long write_word_data(int addr, char cmd, int val)

Read a block of data from a device (cmd is the command or register declaration)
long[] read_block_data(int addr, char cmd)

Send a block of data to a device (cmd is the command or register declaration).
The data block should be maximum 31 Byte.
The function adds a length byte before the data bytes.
write_block_data(int addr, char cmd, long vals[])

Process Call transaction
long process_call(int addr, char cmd, int val)

Block Process Call transaction
long[] block_process_call(int addr, char cmd, long vals[])

Read a block of raw data from a device (cmd is the command or register declaration)
long[] read_i2c_block_data(int addr, char cmd)

Send a block of raw data to a device (cmd is the command or register declaration).
write_i2c_block_data(int addr,char cmd, long vals[])

"""
