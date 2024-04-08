# Copyright 2024 iCampus Wildau.
#
# Licensed under the Apache License, Version 2.0.
# See LICENSE in the project root for license information.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Starts the i2c node."""
    description = LaunchDescription()
    
    bridge_node = Node(
        executable="bridge",
        package="ros_i2c",
    )
    
    description.add_action(bridge_node)
    
    return description
