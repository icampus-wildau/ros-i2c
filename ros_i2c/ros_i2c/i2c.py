# Copyright 2024 iCampus Wildau.
#
# Licensed under the Apache License, Version 2.0.
# See LICENSE in the project root for license information.

from __future__ import annotations

import time
from typing import Callable
from typing import Dict

import rclpy
import rclpy.time
import smbus2 as smbus
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from ros_i2c_interfaces.msg import WriteArray
from ros_i2c_interfaces.msg import WriteByte
from ros_i2c_interfaces.msg import WriteWord
from ros_i2c_interfaces.srv import TryWriteArray
from ros_i2c_interfaces.srv import TryWriteByte
from ros_i2c_interfaces.srv import TryWriteWord


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


class I2CNode(Node):
    """ROS 2 node offering I²C functionality to other ROS nodes.

    The node performs respective write operations on the I2C bus upon receiving
    `Write*` messages on the corresponding `i2c/write*` topics.
    """

    def __init__(self, bus=1):
        super().__init__("i2c_bridge_node")

        self.get_logger().info("Starting the I²C bridge node")
        rclpy.get_default_context().on_shutdown(self.on_shutdown)

        self.allow_simulation = self.declare_parameter(
            "allow_simulation",
            False,
            ParameterDescriptor(
                description="Whether to allow this node to fallback to a simulated I²C bus if no hardware I²C bus is available.",
            ),
        )
        """Whether to allow the use of `BusSim`."""

        self.enable_exception_tracking = self.declare_parameter(
            "enable_exception_tracking",
            True,
            ParameterDescriptor(
                description="Parameter to enable/disable the exception tracking for I²C devices.",
            ),
        )
        """Enable or disable the exception tracking for I²C devices."""

        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.devices: Dict[int, SmbusDevice] = dict()

        # Open I2C Bus
        self.bus: smbus.SMBus
        try:
            self.bus = smbus.SMBus(bus)
            self.get_logger().info("Successfully opened SMbus({})".format(bus))

        except Exception as e:
            self.get_logger().info("{}".format(e))

            if self.allow_simulation.value == True:
                self.bus = BusSim(bus, self)
            else:
                raise e

        # Subscribe to I2C bridge topics.
        self.create_subscription(WriteArray, "i2c/write_array", self.on_write_array, 10)
        self.create_subscription(WriteByte, "i2c/write_byte", self.on_write_byte, 10)
        self.create_subscription(WriteWord, "i2c/write_word", self.on_write_word, 10)

        # Create I2C bridge services.
        self.create_service(TryWriteArray, "i2c/try_write_array", self.on_try_write_array)
        self.create_service(TryWriteByte, "i2c/try_write_byte", self.on_try_write_byte)
        self.create_service(TryWriteWord, "i2c/try_write_word", self.on_try_write_word)

        ################################
        ### LOG INFO ###################

        self.get_logger().info("#### SUBSCRIBER ####")
        for s in self.subscriptions:
            self.get_logger().info(f"Subscribed: {s.topic_name:<25} | Msg-Type: {str(s.msg_type)}")

        self.get_logger().info("#### PUBLISHER ####")
        for s in self.publishers:
            self.get_logger().info(f"Publishing: {s.topic_name:<25} | Msg-Type: {str(s.msg_type)}")

        self.get_logger().info("#### SERVICES ####")
        for s in self.services:
            self.get_logger().info(f"Service: {s.srv_name:<50} | Srv-Type: {str(s.srv_type)}")

        ################################

    ##########################################
    ### Internal Helper Functions
    ##########################################

    @staticmethod
    def _int_to_uint8(x: int):
        return x % 256

    def _get_device(self, address: int) -> SmbusDevice:
        if address not in self.devices:
            self.devices[address] = SmbusDevice(self, address)

        return self.devices[address]

    ##########################################
    ### Internal Callback Functions
    ##########################################

    def on_set_parameters(self, params):
        for param in params:
            if param.name == "exception_logger_enabled":
                self.enable_exception_tracking = param.value
                self.get_logger().info(f"Set exception_logger_enabled to {self.enable_exception_tracking}")

    def on_shutdown(self):
        self.get_logger().info("#### SHUTDOWN ####")
        self.bus.close()

    ##########################################
    ### Subscriber Callback Functions
    ##########################################

    def on_write_array(self, msg: WriteArray):
        """Writes multiple bytes of data to an I²C device, ignoring errors.

        Publishing a `WriteArray` message to this topic will write its `data` to
        the I²C device at `address` using the specified `command`.

        This corresponds to the `write_block_data` I²C command (if `write_length`
        is `True`) or to the `write_i2c_block_data` command (if `False`).
        """
        data: list[int] = []

        for d in msg.data:
            data.append(self._int_to_uint8(int(d)))

        self.get_logger().debug(f"write_array: addr: {msg.address} cmd: {msg.command} data: {data}")
        self._get_device(msg.address).write_array(msg.command, data, msg.write_length)

    def on_write_byte(self, msg: WriteByte):
        """Writes 8 bit of data to an I²C device, ignoring errors.

        Publishing a `WriteByte` message to this topic will write its `data` to
        the I²C device at `address` using the specified `command`.

        This corresponds to the `write_byte_data` I²C command.
        """
        self.get_logger().debug(f"write_byte: addr: {msg.address} cmd: {msg.command} data: {msg.data}")
        self._get_device(msg.address).write_byte(msg.command, msg.data)

    def on_write_word(self, msg: WriteWord):
        """Writes 16 bit of data to an I²C device, ignoring errors.

        Publishing a `WriteWord` message to this topic will write its `data` to
        the I²C device at `address` using the specified `command`.

        This corresponds to the `write_word_data` I²C command.
        """
        self.get_logger().debug(f"write_word: addr: {msg.address} cmd: {msg.command} data: {msg.data}")
        self._get_device(msg.address).write_word(msg.command, msg.data)

    ##########################################
    ### Service Callback Functions
    ##########################################

    def on_try_write_array(
        self, request: TryWriteArray.Request, response: TryWriteArray.Response
    ) -> TryWriteArray.Response:
        """Tries to write multiple bytes of data to an I²C device.

        This service will write the `data` of a `WriteArray` message to the I²C
        device at `address` using the specified `command` and respond with the
        success status (`True` iff successful) of the operation.

        This corresponds to the `write_block_data` I²C command (if `write_length`
        is `True`) or to the `write_i2c_block_data` command (if `False`).
        """
        msg = request.message

        self.get_logger().debug(f"try_write_array: addr: {msg.address} cmd: {msg.command} data: {msg.data}")
        success = self._get_device(msg.address).write_array(msg.command, msg.data, msg.write_length)

        response.success = success

        return response

    def on_try_write_byte(
        self, request: TryWriteByte.Request, response: TryWriteByte.Response
    ) -> TryWriteByte.Response:
        """Tries to write 8 bit of data to an I²C device.

        This service will write the `data` of a `WriteByte` message to the I²C
        device at `address` using the specified `command` and respond with the
        success status (`True` iff successful) of the operation.

        This corresponds to the `write_byte_data` I²C command.
        """
        msg = request.message

        self.get_logger().debug(f"try_write_byte: addr: {msg.address} cmd: {msg.command} data: {msg.data}")
        response.success = self._get_device(msg.address).write_byte(msg.command, msg.data)

        return response

    def on_try_write_word(
        self, request: TryWriteWord.Request, response: TryWriteWord.Response
    ) -> TryWriteWord.Response:
        """Tries to write 16 bit of data to an I²C device.

        This service will write the `data` of a `WriteWord` message to the I²C
        device at `address` using the specified `command` and respond with the
        success status (`True` iff successful) of the operation.

        This corresponds to the `write_word_data` I²C command.
        """
        msg = request.message

        self.get_logger().debug(f"try_write_word: addr: {msg.address} cmd: {msg.command} data: {msg.data}")
        response.success = self._get_device(msg.address).write_word(msg.command, msg.data)

        return response


class SmbusDevice:
    """Helper class representing a device on the I²C bus, taking care of retries
    and exceptions.
    """

    def __init__(
        self, ros_node: I2CNode, device_address: int, exception_count: int = 3, exception_timeout_sec: int = 10
    ) -> None:
        """Create a new I²C device.

        The exception tracking can be enabled or disabled by setting the
        `enable_exception_tracking` parameter on the parent node.

        Parameters
        ----------
        device_address : int
            The address of the i2c device.

        exception_count : int, optional
            How many subsequent exceptions are allowed before the device is
            considered dead, by default 3.

        exception_timeout_sec : int, optional
            How long to wait before trying again, by default 10.
        """

        self._node = ros_node
        """The parent node of the exception logger"""

        self.bus = ros_node.bus
        """The bus to write to"""

        self.address = device_address
        """The address of the I²C device the exception logger is for"""

        self.last_time_tried: int = 0
        """The last time the I²C device was last tried"""

        self.current_exception_count = 1
        """How many subsequent exceptions are allowed before the I²C device is considered dead"""

        self._max_exception_count = exception_count
        self._exception_timeout_sec = exception_timeout_sec

    def is_alive(self):
        """
        If the exception count is reached, the I²C device is considered dead
        After the given timeout, the I²C device is considered alive again.
        """
        if not self._node.enable_exception_tracking:
            return True

        return (time.time() - self.last_time_tried > self._exception_timeout_sec) or (
            self.current_exception_count <= self._max_exception_count
        )

    def _try_write(
        self,
        bus_write_callback: Callable[[int, int, int | list[int]], None],
        command: int,
        data: int | list[int],
        retries: int = 2,
    ):
        """Try to write data to the I²C device.
        Handles exceptions and retries.

        Parameters
        ----------
        bus_write_callback : Callable[[int, int, int  |  list[int]], None]
            The bus write function to call
        command : int
            Command to send to the device.
        data : int | list[int]
            Data to send to the device.
        retries : int, optional
            Count of immediate retries, by default 2
        """
        for _ in range(retries):
            try:
                if self.is_alive():
                    bus_write_callback(self.address, command, data)
                    # Reset the exception counter if the write was successful
                    self.current_exception_count = 0
                    return True
                else:
                    return False
            except Exception as e:
                self._node.get_logger().error(f"I²C device @ {self.address} has a timeout at writing data.")
                self.last_time_tried = time.time()
                self.current_exception_count += 1

                self._node.get_logger().error(str(e) + " ... Trying again")
                time.sleep(0.05)

        return False

    def write_array(self, command: int, data: list[int], write_length=True):
        """Write a block of data to the I²C device.

        Parameters
        ----------
        command : int
            Command to send to the device.
        data : list[int]
            Data list to send to the device.
        write_length : bool
            Whether to prefix the data with a byte specifying its length.
        """
        if write_length:
            return self._try_write(self.bus.write_block_data, command, data)
        else:
            return self._try_write(self.bus.write_i2c_block_data, command, data)

    def write_byte(self, command: int, data: int):
        """Write a byte to the I²C device.

        Parameters
        ----------
        command : int
            Command to send to the device.
        data : int
            Data to send to the device.
        """
        return self._try_write(self.bus.write_byte_data, command, data)

    def write_word(self, command: int, data: int):
        """Write a word to the I²C device.

        Parameters
        ----------
        command : int
            Command to send to the device.
        data : int
            Data to send to the device.
        """
        return self._try_write(self.bus.write_word_data, command, data)


class BusSim(object):
    """Simulator for the SMBus class."""

    def __init__(self, bus_num: int, ros_node: I2CNode):

        self.num = bus_num
        self.node = ros_node
        self.log = self.node.get_logger()

        self.log.warn("####### ERROR OPENING SMbus #######")
        self.log.warn("Starting SMbus Simulator ({})".format(self.num))

    def write_block_data(self, addr, cmd, data):
        self.log.debug("Write @ {} CMD: {} Data: {}".format(addr, cmd, data))

    def write_byte_data(self, addr, cmd, data):
        self.log.debug("Write @ {} CMD: {} Data: {}".format(addr, cmd, data))

    def write_word_data(self, addr, cmd, data):
        self.log.debug("Write @ {} CMD: {} Data: {}".format(addr, cmd, data))
