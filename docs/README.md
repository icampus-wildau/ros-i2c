<!-- MD+:META
title = "The documentation for the ROS I²C package."
-->

# Usage Examples

<!-- MD+:include.example
header = 'Basic Publishing Example'
level = 1
path = '../examples/publishing.py'
-->
# Basic Publishing Example

This script demonstrates how to publish i2c messages to the ros-i2c package.

```python
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
```

Source of the above code: [examples/publishing.py](./../examples/publishing.py).
<!-- MD+FIN:include.example -->

# ROS I²C Specification

<!-- MD+:ros.launchs
header = '# ROS Launch Scripts'
level = 2
-->
## ROS Launch Scripts

|Info                |Script                                                         |
|--------------------|---------------------------------------------------------------|
|Starts the i2c node.|[ros_i2c/launch/i2c.launch.py](../ros_i2c/launch/i2c.launch.py)|
<!-- MD+FIN:ros.launchs -->

<!-- MD+:ros.nodes
header = '# ROS Nodes'
level = 2
only_commented_publishers = True
only_commented_subscriptions = True
only_commented_services = True
include_parameters = True
-->
## ROS Nodes

|Package|Name                  |Info                                                     |Script                                        |
|-------|----------------------|---------------------------------------------------------|----------------------------------------------|
|ros_i2c|[bridge](#bridge-node)|ROS 2 node offering I²C functionality to other ROS nodes.|[ros_i2c.bridge](../ros_i2c/ros_i2c/bridge.py)|

### `bridge` Node

ROS 2 node offering I²C functionality to other ROS nodes.

The node performs respective write operations on the I2C bus upon receiving
`Write*` messages on the corresponding `i2c/write*` topics.

**Parameters of this node**

|Name                     |Default|Info                                                                                              |
|-------------------------|-------|--------------------------------------------------------------------------------------------------|
|allow_simulation         |False  |Whether to allow this node to fallback to a simulated I²C bus if no hardware I²C bus is available.|
|enable_exception_tracking|True   |Parameter to enable/disable the exception tracking for I²C devices.                               |

**Publisher, Subscriber and Services of this node**

|Topic                                       |Type                             |Kind        |Info                                                            |
|--------------------------------------------|---------------------------------|------------|----------------------------------------------------------------|
|[`i2c/try_write_array`](#i2ctry_write_array)|[`TryWriteArray`](#trywritearray)|Service     |Tries to write multiple bytes of data to an I²C device.         |
|[`i2c/try_write_byte`](#i2ctry_write_byte)  |[`TryWriteByte`](#trywritebyte)  |Service     |Tries to write 8 bit of data to an I²C device.                  |
|[`i2c/try_write_word`](#i2ctry_write_word)  |[`TryWriteWord`](#trywriteword)  |Service     |Tries to write 16 bit of data to an I²C device.                 |
|[`i2c/write_array`](#i2cwrite_array)        |[`WriteArray`](#writearray)      |Subscription|Writes multiple bytes of data to an I²C device, ignoring errors.|
|[`i2c/write_byte`](#i2cwrite_byte)          |[`WriteByte`](#writebyte)        |Subscription|Writes 8 bit of data to an I²C device, ignoring errors.         |
|[`i2c/write_word`](#i2cwrite_word)          |[`WriteWord`](#writeword)        |Subscription|Writes 16 bit of data to an I²C device, ignoring errors.        |

#### `i2c/try_write_array`
Tries to write multiple bytes of data to an I²C device.

This service will write the `data` of a `WriteArray` message to the I²C
device at `address` using the specified `command` and respond with the
success status (`True` iff successful) of the operation.

This corresponds to the `write_block_data` I²C command (if `write_length`
is `True`) or to the `write_i2c_block_data` command (if `False`).

#### `i2c/try_write_byte`
Tries to write 8 bit of data to an I²C device.

This service will write the `data` of a `WriteByte` message to the I²C
device at `address` using the specified `command` and respond with the
success status (`True` iff successful) of the operation.

This corresponds to the `write_byte_data` I²C command.

#### `i2c/try_write_word`
Tries to write 16 bit of data to an I²C device.

This service will write the `data` of a `WriteWord` message to the I²C
device at `address` using the specified `command` and respond with the
success status (`True` iff successful) of the operation.

This corresponds to the `write_word_data` I²C command.

#### `i2c/write_array`
Writes multiple bytes of data to an I²C device, ignoring errors.

Publishing a `WriteArray` message to this topic will write its `data` to
the I²C device at `address` using the specified `command`.

This corresponds to the `write_block_data` I²C command (if `write_length`
is `True`) or to the `write_i2c_block_data` command (if `False`).

#### `i2c/write_byte`
Writes 8 bit of data to an I²C device, ignoring errors.

Publishing a `WriteByte` message to this topic will write its `data` to
the I²C device at `address` using the specified `command`.

This corresponds to the `write_byte_data` I²C command.

#### `i2c/write_word`
Writes 16 bit of data to an I²C device, ignoring errors.

Publishing a `WriteWord` message to this topic will write its `data` to
the I²C device at `address` using the specified `command`.

This corresponds to the `write_word_data` I²C command.
<!-- MD+FIN:ros.nodes -->

<!-- MD+:ros.interfaces
header = '# ROS Interface Definitions'
level = 2
-->
## ROS Interface Definitions

|Name                             |Type   |Package           |
|---------------------------------|-------|------------------|
|[`WriteArray`](#writearray)      |Message|ros_i2c_interfaces|
|[`WriteByte`](#writebyte)        |Message|ros_i2c_interfaces|
|[`WriteWord`](#writeword)        |Message|ros_i2c_interfaces|
|[`TryWriteArray`](#trywritearray)|Service|ros_i2c_interfaces|
|[`TryWriteByte`](#trywritebyte)  |Service|ros_i2c_interfaces|
|[`TryWriteWord`](#trywriteword)  |Service|ros_i2c_interfaces|

### Message definitions of ros_i2c_interfaces

#### `WriteArray`

```python
# The I2C address of the device to write to.
uint8 address

# The command to sent to the device.
uint8 command

# The data/payload (byte array) to sent to the device.
uint8[] data

# Whether to prefix the data with a single byte specifying its length.
#
# Iff `true` (default), the length byte will be included.
bool write_length true
```

Source: [ros_i2c_interfaces/msg/WriteArray.msg](../ros_i2c_interfaces/msg/WriteArray.msg)

#### `WriteByte`

```python
# The I2C address of the device to write to.
uint8 address

# The command to sent to the device.
uint8 command

# The data/payload (single byte) to sent to the device.
uint8 data
```

Source: [ros_i2c_interfaces/msg/WriteByte.msg](../ros_i2c_interfaces/msg/WriteByte.msg)

#### `WriteWord`

```python
# The I2C address of the device to write to.
uint8 address

# The command to sent to the device.
uint8 command

# The data/payload (2 bytes) to sent to the device.
uint16 data
```

Source: [ros_i2c_interfaces/msg/WriteWord.msg](../ros_i2c_interfaces/msg/WriteWord.msg)

### Service definitions of ros_i2c_interfaces

#### `TryWriteArray`

```python
# A `WriteArray` message specifying the data to write.
WriteArray message
---
# A flag indicating the success of this operation if set. Defaults to `true`.
bool success true
```

Source: [ros_i2c_interfaces/srv/TryWriteArray.srv](../ros_i2c_interfaces/srv/TryWriteArray.srv)

#### `TryWriteByte`

```python
# A `WriteByte` message specifying the data to write.
WriteByte message
---
# A flag indicating the success of this operation if set. Defaults to `true`.
bool success true
```

Source: [ros_i2c_interfaces/srv/TryWriteByte.srv](../ros_i2c_interfaces/srv/TryWriteByte.srv)

#### `TryWriteWord`

```python
# A `WriteByte` message specifying the data to write.
WriteByte message
---
# A flag indicating the success of this operation if set. Defaults to `true`.
bool success true
```

Source: [ros_i2c_interfaces/srv/TryWriteWord.srv](../ros_i2c_interfaces/srv/TryWriteWord.srv)
<!-- MD+FIN:ros.interfaces -->
