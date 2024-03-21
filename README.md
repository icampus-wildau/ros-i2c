# ROS I²C

**A ROS 2 package for low level hardware communication via I²C.**

This package provides a ROS 2 node that allows for communication with I²C devices connected to the host system. Data can be simply transmitted to I²C peripherals, such as sensors or actuators, by publishing ROS 2 messages to the I²C node.

# Getting Started using [pakk](https://github.com/iCampus-Wildau/pakk)

This guide will help you to install the ROS I²C package and run the I²C node on your system using the [pakk](https://github.com/iCampus-Wildau/pakk) package manager, which automates the installation and management of ROS 2 packages. If you prefer a more manual approach, please refer to the [Getting Started from Scratch](#getting-started-from-scratch) section. However, using pakk is recommended for a more convenient management of ROS 2 packages.

## Installation

MD+:TODO: Auto generate with markdwon plus if pakk.cfg file is present

To install the ROS I²C package using pakk, simply run the following command:

```bash
pakk install ros-i2c
```

## Usage

After the installation completes, you can start the I²C node by using one of the following commands:

```bash
pakk start ros-i2c  # Start the I²C node until the system is rebooted, or ...
pakk enable ros-i2c  # ... start the I²C node now and on every system boot.
```

See the [Examples](#examples) section for more information on how to use the I²C node.

# Getting Started from Scratch 

This guide will help you to install the ROS I²C package and run the I²C node on your system. If you prefer a more automated/managed approach, please refer to the [Getting Started using pakk](#getting-started-using-pakk) section, which uses the [pakk](https://github.com/iCampus-Wildau/pakk) package manager.

## Installation 

MD+:TODO: Auto generate by parsing the pakk.cfg file

## Usage 

MD+:TODO: Auto generate by parsing the repo

After you installed the I²C package, you can run the I²C node in your terminal by using one of the following commands:

```bash
ros2 run ros_i2c i2c  # Run the I²C node directly, or ...
ros2 launch ros_i2c i2c.launch.py  # ... by using a launch file.
```

See the [Examples](#examples) section for more information on how to use the I²C node.

# Examples

***Wie wäre es mit einem Beispieleabschnitt? Sonst müsste man das bei beiden Getting Started Abschnitten machen, also doppelt. und hier könnte man auch direkt hinspringen, wenn mans schon installiert hat.***

> [!CAUTION]
> Be careful when using the I²C node, as it allows for direct communication with hardware peripherals. Sending the wrong data to the wrong address can damage your hardware or lead to otherwise unexpected behavior. For neither the authors nor the maintainers of this package can be held responsible for.

To send data to an I²C peripheral, you can publish a message to the I²C node. For example, to transmit the byte `0x42` using the command `0x01` to the I²C address `0x42`, you could use the following command:

```bash
ros2 topic pub /i2c/write8 "{address: 0x42, command: 0x01, data: 0x42}"
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
