# Copyright 2024 iCampus Wildau.
#
# Licensed under the Apache License, Version 2.0.
# See LICENSE in the project root for license information.

"""
This ROS 2 node offers I2C functionality to other ROS nodes.

The node performs respective write operations on the I2C bus upon receiving
`Write*` messages on the corresponding `i2c/write*` topics.
"""

import time

from typing import Dict

import smbus2 as smbus

import rclpy

from rclpy.node import Node

from ros_i2c_interfaces.msg import Write8, Write16, WriteArray


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
        self.bus: smbus.SMBus
        try:
            self.bus = smbus.SMBus(bus)
            self.get_logger().info("Successfully opened SMbus({})".format(bus))
        except Exception as e:
            self.get_logger().info("{}".format(e))
            self.bus = BusSim(bus, self)

        # Subscribe to I2C Bridge Topics

        self.sub_i2c_8 = self.create_subscription(Write8, "i2c/write8", self.on_write_8, 10)
        self.sub_i2c_16 = self.create_subscription(Write16, "i2c/write16", self.on_write_16, 10)
        self.sub_i2c_arr = self.create_subscription(WriteArray, "i2c/write_array", self.on_write_array, 10)

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

    def on_write_16(self, msg: Write16):
        """
        Send 16 bit data over I2C.

        Parameters
        ----------
        msg : Write16
            I2C message with address, command and data.
        """
        self.get_logger().debug("I2C write16:    addr: {} cmd: {} data: {}".format(str(msg.address), str(msg.command), str(msg.data)))
        try:
            self.bus.write_word_data(msg.address, msg.command, msg.data)
        except Exception as e:
            self.get_logger().error(str(e))

    def on_write_8(self, msg: Write8):
        """
        Send 8 bit data over I2C.

        Parameters
        ----------
        msg : Write8
            I2C message with address, command and data.
        """
        self.get_logger().debug("I2C write8:     addr: {} cmd: {} data: {}".format(str(msg.address), str(msg.command), str(msg.data)))
        try:
            self.bus.write_byte_data(msg.address, msg.command, msg.data)
        except Exception as e:
            self.get_logger().error(str(e))

    def on_write_array(self, msg: WriteArray):
        """
        Send an array of data over I2C.

        Parameters
        ----------
        msg : WriteArray
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
