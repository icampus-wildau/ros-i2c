from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    """Starts the i2c node."""
    
    ld = LaunchDescription()
    i2c_node = Node(
        package="ros_i2c",
        executable="bridge",
        # remappings=[
        #     ("system/i2c/write8", "system/i2c/write8"),
        #     ("system/i2c/write16", "system/i2c/write16"),
        #     ("system/i2c/writeArray", "system/i2c/writeArray")
        # ]
    )
    ld.add_action(i2c_node)
    return ld