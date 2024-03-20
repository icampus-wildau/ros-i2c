# ROS I²C

ROS 2 Package for low level hardware communication via I²C.
The started node act as I²C masters in the system.

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
- scanning for connected slaves
- receiving data from slaves

# Compatibility

The ROS I²C package is designed and tested to work well with ROS 2 (Humble) on a Raspberry Pi 4. However, it should be compatible with any platform supported by both ROS 2 and the [smbus2](https://github.com/kplindegaard/smbus2) Python library.

# Contents of this Repository

MD+:TODO: Auto-Generate by parsing the repo

# Documentation

A more detailed documentation can be found at [`docs/README.md`](docs/README.md).

# Questions/Issues
Describe where questions about this software/project may be asked and how GitHub issues should be used. Here would be a good place to point users of the software/product to helpful community forums and alike.

# Contributing
Explain how others may contribute to this software/project. This might include linking the contributer guidelines and a code of conduct, as well as, pointing new contributers to „good first issues“ (if such concept applies).

# Related projects
Here is a list of related projects...
–	link to project 1
–	link to another project

# License
This project is licensed under the Apache License 2.0. For details, please see the LICENSE file. By contributing to this project, you agree to abide by the terms and conditions of the Apache License 2.0.
