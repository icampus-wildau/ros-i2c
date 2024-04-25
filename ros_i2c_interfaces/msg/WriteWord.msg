# Copyright 2024 iCampus Wildau.
#
# Licensed under the Apache License, Version 2.0.
# See LICENSE in the project root for license information.

# The I2C address of the device to write to.
uint8 address

# The command to sent to the device.
uint8 command

# The data/payload (2 bytes) to sent to the device.
uint16 data
