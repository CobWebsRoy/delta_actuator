cmake_minimum_required(VERSION 3.8)
project(actuator_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ActuatorState.msg"
    "msg/ActuatorRequest.msg"
    "msg/ActuatorCommand.msg"
    "msg/ActuatorSensor.msg"
)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()
