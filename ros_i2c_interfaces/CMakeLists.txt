cmake_minimum_required(VERSION 3.5)
project(ros_i2c_interfaces)

# Default to C99.
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)

# Generate ROS interfaces.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WriteArray.msg" "msg/WriteByte.msg" "msg/WriteWord.msg"
  "srv/TryWriteArray.srv" "srv/TryWriteByte.srv" "srv/TryWriteWord.srv")

ament_package()
