# Delta_Actuator

This ROS2 code is for a delta arm actuator, initially intended for a UAV to pick up items anywhere underneath it.

The actuator_logic node subscribes to ActuatorRequests, and publishes to ActuatorCommands.
The servo_controller node subscribes to ActuatorCommands, and physically actuates servos.
