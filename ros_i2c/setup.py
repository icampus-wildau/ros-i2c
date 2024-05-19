# Copyright 2024 iCampus Wildau.
#
# Licensed under the Apache License, Version 2.0.
# See LICENSE in the project root for license information.

from __future__ import annotations

from setuptools import setup
from setuptools.glob import glob

name = "ros_i2c"

setup(
    name=name,
    version="1.0.0",
    packages=[name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{name}"]),
        (f"share/{name}", ["package.xml"]),
        (f"share/{name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Valentin Schröter",
    maintainer_email="vasc9380@th-wildau.de",
    description="A ROS 2 package for low level hardware communication via I²C.",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "i2c = ros_i2c.i2c:main",
        ],
    },
)
