# ROS I²C

**A ROS 2 package for low level hardware communication via I²C.**

This package provides a ROS 2 node that allows for communication with I²C devices connected to the host system. Data can be simply transmitted to I²C peripherals, such as sensors or actuators, by publishing ROS 2 messages to the I²C node.

# Getting Started from Scratch 
Provide a quick start for someone (completely) new to this project. This may include aquiring a copy of the software/product, installation and how to run it. Also point users to documentation on more advanced topics.

## Installation 

MD+:TODO: Auto generate by parsing the pakk.cfg file

## Usage 

MD+:TODO: Auto generate by parsing the repo
```bash
ros2 run ros_i2c i2c
ros2 launch ros_i2c i2c.launch.py
```

# Getting Started using [pakk](https://github.com/iCampus-Wildau/pakk)

## Installation

MD+:TODO: Auto generate with markdwon plus if pakk.cfg file is present

```bash
pakk install ...
```

## Usage

```bash
pakk start / enable ros-i2c
```

# Features

Features of the I²C-Node:
- Transmit 8-bit data
- Transmit 16-bit data
- Transmit byte arrays (up to 30 bytes)

Future work:
- scanning for connected peripherals
- receiving data from peripherals

# Compatibility

The ROS I²C package is designed and tested to work well with ROS 2 (Humble) on a Raspberry Pi 4 (Model B). However, it should be compatible with any platform supported by both ROS 2 and the [smbus2](https://github.com/kplindegaard/smbus2) Python library.

# Contents of this Repository

MD+:TODO: Auto-Generate by parsing the repo

# Documentation

A more detailed documentation can be found at [`docs/README.md`](docs/README.md).

# Questions/Issues

If you encounter any problems or have any questions, please open an issue on the GitHub repository.

# Contributing
Explain how others may contribute to this software/project. This might include linking the contributer guidelines and a code of conduct, as well as, pointing new contributers to „good first issues“ (if such concept applies).

# Related Projects

The I²C communication is handled by the [smbus2](https://github.com/kplindegaard/smbus2) Python library.

# License

This project is licensed under the Apache License 2.0. For details, please see the [LICENSE](LICENCE) file. By contributing to this project, you agree to abide by the terms and conditions of the Apache License 2.0.
