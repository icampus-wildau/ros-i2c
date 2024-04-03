<!-- MD+:META
title = "The documentation for the ROS I²C package."
-->

# Usage Examples

<!-- MD+:include.example 
header = 'Basic Publishing Example'
level = 1
path = '../examples/publishing.py'
-->
# [Basic Publishing Example](../examples/publishing.py)

This script demonstrates how to publish i2c messages to the ros-i2c package.

```python
# Import the i2c interfaces
from ros_i2c_interfaces.msg import Write8, Write16, WriteArray

# Create publishers for the i2c messages
pub_i2c_8 = node.create_publisher(Write8, "i2c/write8", 10)
pub_i2c_16 = node.create_publisher(Write16, "i2c/write16", 10) 
pub_i2c_array = node.create_publisher(WriteArray, "i2c/write_array", 10)

# Methods to publish i2c messages
def publish_8_bit(self, cmd: int, data: int):
    o = Write8()
    o.address = self.i2c_address
    o.command = cmd
    o.data = data
    
    pub_i2c_8.publish(o)

def publish_16_bit(self, cmd: int, data: int):
    o = Write16()
    o.address = self.i2c_address
    o.command = cmd
    o.data = data
    
    pub_i2c_16.publish(o)

def publish_array(self, cmd: int, data: list[int]):
    o = WriteArray()
    o.address = self.i2c_address
    o.command = cmd
    o.data = data
    
    pub_i2c_array.publish(o)
```


<!-- MD+FIN:include.example -->

# ROS I²C Specification

<!-- MD+:ros.launchs 
header = '# ROS Launch Scripts'
level = 2
-->
## ROS Launch Scripts

|Name|        Info        |                Script                |
|----|--------------------|--------------------------------------|
| i2c|Starts the i2c node.|[i2c](../ros_i2c/launch/i2c.launch.py)|
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

|Package|      Name      |                            Info                            |                    Script                    |
|-------|----------------|------------------------------------------------------------|----------------------------------------------|
|ros_i2c|[i2c](#i2c-node)|This ROS 2 node offers I2C functionality to other ROS nodes.|[ros_i2c.bridge](../ros_i2c/ros_i2c/bridge.py)|

### `i2c` Node

This ROS 2 node offers I2C functionality to other ROS nodes.

The node performs respective write operations on the I2C bus upon receiving
`Write*` messages on the corresponding `i2c/write*` ROS 2 topics.

**Publisher, Subscriber and Services of this node**

|Topic                               |Type                       |Kind        |Info                           |
|------------------------------------|---------------------------|------------|-------------------------------|
|[`i2c/write16`](#i2cwrite16)        |[`Write16`](#write16)      |Subscription|Send 16 bit data over I2C.     |
|[`i2c/write8`](#i2cwrite8)          |[`Write8`](#write8)        |Subscription|Send 8 bit data over I2C.      |
|[`i2c/write_array`](#i2cwrite_array)|[`WriteArray`](#writearray)|Subscription|Send an array of data over I2C.|

#### `i2c/write8`
```
Send 8 bit data over I2C.

Parameters
----------
msg : Write8
    I2C message with address, command and data.
```

#### `i2c/write16`
```
Send 16 bit data over I2C.

Parameters
----------
msg : Write16
    I2C message with address, command and data.
```

#### `i2c/write_array`
```
Send an array of data over I2C.

Parameters
----------
msg : Write16
    I2C message with address, command and an data array.
```
<!-- MD+FIN:ros.nodes -->

<!-- MD+:ros.interfaces 
header = '# ROS Interface Definitions'
level = 2
-->
## ROS Interface Definitions

|            Name           |  Type |      Package     |
|---------------------------|-------|------------------|
|   [`Write16`](#write16)   |Message|ros_i2c_interfaces|
|    [`Write8`](#write8)    |Message|ros_i2c_interfaces|
|[`WriteArray`](#writearray)|Message|ros_i2c_interfaces|

### Message definitions of ros_i2c_interfaces

#### [`Write16`](../ros_i2c_interfaces/msg/Write16.msg)

```python
# The I2C address of the peripheral device to write to.
uint8 address

# The command to sent to the peripheral device.
uint8 command

# The data/payload (2 bytes) to sent to the peripheral device.
uint16 data
```


#### [`Write8`](../ros_i2c_interfaces/msg/Write8.msg)

```python
# The I2C address of the peripheral device to write to.
uint8 address

# The command to sent to the peripheral device.
uint8 command

# The data/payload (single byte) to sent to the peripheral device.
uint8 data
```


#### [`WriteArray`](../ros_i2c_interfaces/msg/WriteArray.msg)

```python
# The I2C address of the peripheral device to write to.
uint8 address

# The command to sent to the peripheral device.
uint8 command

# The data/payload (byte array) to sent to the peripheral device.
uint8[] data
```

<!-- MD+FIN:ros.interfaces -->