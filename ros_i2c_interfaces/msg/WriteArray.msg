# Copyright 2024 iCampus Wildau
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# The I2C address of the peripheral device to write to.
uint8 address

# The command to sent to the peripheral device.
uint8 command

# The data/payload (byte array) to sent to the peripheral device.
uint8[] data
