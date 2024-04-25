# ROS I²C

**A ROS 2 package for low level hardware communication via I²C.**

This package provides a ROS 2 node that allows for communication with I²C devices connected to the host system. Data can be simply transmitted to I²C peripherals, such as micro controllers or actuators, by publishing ROS 2 messages to the I²C node.

> [!CAUTION]
> Be careful when using the I²C node, as it allows for direct communication with hardware peripherals. Sending the wrong data to the wrong address can damage your hardware or lead to otherwise unexpected behavior, for which neither the authors nor the maintainers of this package can be held responsible for.

# Features & Compatibility

Features of the I²C-Node:
- Transmit 8-bit data
- Transmit 16-bit data
- Transmit byte arrays (up to 30 bytes)

Future work:
- scanning for connected peripherals
- receiving data from peripherals

> [!NOTE]
> The ROS I²C package is designed and tested to work well with **ROS 2 (Humble)** on a **Raspberry Pi 4 (Model B)**. However, it should be compatible with any platform supported by both ROS 2 and the [smbus2](https://github.com/kplindegaard/smbus2) Python library.

<!-- MD+:generate.getting_started.pakk 
header = '# Getting Started using [pakk](https://github.com/iCampus-Wildau/pakk)'
level = 1
installation = True
usage = True
-->
# Getting Started using [pakk](https://github.com/iCampus-Wildau/pakk)
Using [pakk](https://github.com/iCampus-Wildau/pakk) package manager is recommended for automating the installation and management of ROS 2 packages.

Installation with pakk:
```bash
pakk install icampus-wildau/ros-i2c
```

After the installation completes, start the ros-i2c package:
```bash
pakk start ros-i2c  # Run as a service until a reboot / manual stop, or ...
pakk enable ros-i2c  # ... start it now and on every system boot.  
```

<!-- MD+FIN:generate.getting_started.pakk -->

See the [Examples](#usage-examples) section for more information on how to use the I²C node.

# Getting Started from Scratch 

If you prefer to install the package manually, follow the instructions below.


<!-- MD+:TODO: Auto generate by parsing the pakk.cfg file -->

Clone and build the package:

```bash
# Clone the repository
cd ~/ros2_ws/src   # Go to your ROS workspace sources.
git clone https://github.com/iCampus-Wildau/ros-i2c.git

# Install the required dependencies
cd ros-i2c  # Go to the cloned repository.
python3 -m pip install -r requirements.txt

# Build the package
colcon build --packages-select ros_i2c ros_i2c_interfaces
```

<!-- ## Usage  -->

<!-- MD+:TODO: Auto generate by parsing the repo -->

After the installation is completed, run the I²C node:

```bash
ros2 run ros_i2c i2c  # Run the I²C node directly, or ...
ros2 launch ros_i2c i2c.launch.py  # ... by using a launch file.
```

See the [Examples](#usage-examples) section for more information on how to use the I²C node.

# Usage Examples

To send data to an I²C peripheral, you can publish a message to the I²C node. For example, to transmit the byte `0x42` using the command `0x01` to the I²C address `0x42`, you could use the following command:

```bash
ros2 topic pub /i2c/write8 "{address: 0x42, command: 0x01, data: 0x42}"
```

You can also send 16-bit data using the `write16` topic, or multiple bytes using the `write_array` topic. For more information, please refer to the [Documentation](docs/README.md).

<!-- MD+:generate.content 
header = '# Contents of this Repository'
level = 1
dirs = True
md_files = False
-->
# Contents of this Repository

|Directory                                 |Content                                                                  |
|------------------------------------------|-------------------------------------------------------------------------|
|[`docs`](docs)                            |The documentation for the ROS I²C package.                               |
|[`examples`](examples)                    |Basic usage examples for the package.                                    |
|[`ros_i2c`](ros_i2c)                      |Python package containing the ROS I²C nodes.                             |
|[`ros_i2c_interfaces`](ros_i2c_interfaces)|ROS interface package containing the I²C message and service definitions.|
<!-- MD+FIN:generate.content -->

# Documentation

A more detailed documentation can be found at [`docs/README.md`](docs/README.md).

# Questions/Issues

If you encounter any problems, please [open an issue](https://github.com/icampus-wildau/ros-i2c/issues) on the GitHub repository. If you have any questions, feel free to ask them in the [GitHub Discussions](https://github.com/icampus-wildau/ros-i2c/discussions) section.

# Contributing

Contributions to extend the functionality or to solve existing problems are welcome! Requirements for pull requests are:
- All code is tested
- Naming is consistent with project naming
- Commits are squashed and contain a clear commit message describing what functionality is added.

# Related Projects

The I²C communication is handled by the [smbus2](https://github.com/kplindegaard/smbus2) Python library.

# License

This project is licensed under the Apache License 2.0. For details, please see the [LICENSE](LICENCE) file. By contributing to this project, you agree to abide by the terms and conditions of the Apache License 2.0.
