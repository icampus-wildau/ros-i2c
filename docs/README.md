# ROS I²C Documentation

#MD+:ros.complete("..")

<!-- MD+:ros.launchs 
header = '# ROS Launch Scripts'
-->
# ROS Launch Scripts

|Name|        Info        |                Script                |
|----|--------------------|--------------------------------------|
| i2c|Starts the i2c node.|[i2c](..\ros_i2c\launch\i2c.launch.py)|
<!-- MD+FIN:ros.launchs -->

<!-- MD+:ros.nodes 
header = '# ROS Nodes'
only_commented_publishers = True
only_commented_subscriptions = True
only_commented_services = True
include_parameters = True
-->
# ROS Nodes

|Package|      Name      |                             Info                             |                   Script                   |
|-------|----------------|--------------------------------------------------------------|--------------------------------------------|
|ros_i2c|[i2c](#i2c-node)|ROS Node to offer I2C functionality to the rest of the system.|[ros_i2c.bridge](\ros_i2c\ros_i2c\bridge.py)|

## `i2c` Node

ROS Node to offer I2C functionality to the rest of the system.

This node subscribes ROS-Messages to send data over I2C.
This node is for the Raspberry Pi 4B.

I2C: 
http://www.netzmafia.de/skripten/hardware/RasPi/RasPi_I2C.html 
and
https://raspberry-projects.com/pi/programming-in-python/i2c-programming-in-python/using-the-i2c-interface-2 

**Publisher, Subscriber and Services of this node**

|Topic                             |Type                       |Kind        |Info                           |
|----------------------------------|---------------------------|------------|-------------------------------|
|[`i2c/write16`](#i2cwrite16)      |[`Write16`](#write16)      |Subscription|Send 16 bit data over I2C.     |
|[`i2c/write8`](#i2cwrite8)        |[`Write8`](#write8)        |Subscription|Send 8 bit data over I2C.      |
|[`i2c/writeArray`](#i2cwritearray)|[`WriteArray`](#writearray)|Subscription|Send an array of data over I2C.|

### `i2c/write8`
```
Send 8 bit data over I2C.

Parameters
----------
msg : Write8
    I2C message with address, command and data.
```

### `i2c/write16`
```
Send 16 bit data over I2C.

Parameters
----------
msg : Write16
    I2C message with address, command and data.
```

### `i2c/writeArray`
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
-->
# ROS Interface Definitions

|            Name           |  Type |      Package     |
|---------------------------|-------|------------------|
|   [`Write16`](#write16)   |Message|ros_i2c_interfaces|
|    [`Write8`](#write8)    |Message|ros_i2c_interfaces|
|[`WriteArray`](#writearray)|Message|ros_i2c_interfaces|

## Message definitions of ros_i2c_interfaces

### [`Write16`](\ros_i2c_interfaces\msg\Write16.msg)

```python
# Address of the slave device
uint8 address
# Command sent to the slave device
uint8 command
# Data sent to the slave device (two bytes)
uint16 data
```


### [`Write8`](\ros_i2c_interfaces\msg\Write8.msg)

```python
# Address of the slave device
uint8 address
# Command sent to the slave device
uint8 command
# Data sent to the slave device (single byte)
uint8 data
```


### [`WriteArray`](\ros_i2c_interfaces\msg\WriteArray.msg)

```python
# Address of the slave device
uint8 address
# Command sent to the slave device
uint8 command
# Data sent to the slave device (byte array)
uint8[] data
```

<!-- MD+FIN:ros.interfaces -->