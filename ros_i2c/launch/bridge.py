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
