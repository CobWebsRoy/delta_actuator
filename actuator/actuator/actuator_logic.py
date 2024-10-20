import rclpy
from rclpy.node import Node
from actuator_msgs.msg import ActuatorState, ActuatorRequest, ActuatorCommand, ActuatorSensor
import time

class ActuatorLogic(Node):
    def __init__(self):
        super().__init__('actuator_logic')
        
        # Publishers
        self.state_pub = self.create_publisher(ActuatorState, '/actuator_states', 10)
        self.command_pub = self.create_publisher(ActuatorCommand, '/actuator_commands', 10)

        # Subscribers
        self.request_sub = self.create_subscription(
            ActuatorRequest,
            '/actuator_requests',
            self.request_callback,
            10
        )
        self.sensor_sub = self.create_subscription(
            ActuatorSensor,
            '/actuator_sensor',
            self.sensor_callback,
            10
        )

        self.get_logger().info("Actuator Logic Node has been started.")
        
        # Initialize sensor state (simulated)
        self.sensor_state = 0  # Default state

    def request_callback(self, msg):
        
        self.get_logger().info(f'Received request: {msg.request_type}, target_x: {msg.target_x}, target_y: {msg.target_y}')
        
        if msg.request_type == 0:
            self.handle_request_type_0()
        elif msg.request_type == 1:
            self.handle_request_type_1()
        elif msg.request_type == 2:
            self.handle_request_type_2(msg.target_x, msg.target_y)
        elif msg.request_type == 3:
            self.handle_request_type_3()
        else:
            self.get_logger().warn(f"Unknown request_type: {msg.request_type}")
            
    def sensor_callback(self, msg):
        self.get_logger().info(f'Received sensor value: {msg.value}')
        self.sensor_state = msg.value  # Update the sensor state with the received value

    def handle_request_type_0(self):
        # Turn Off Relay
        command = ActuatorCommand()
        command.x = 0.0  # Ensure this is a float
        command.y = 0.0  # Ensure this is a float
        command.z = -30.0  # Ensure this is a float
        command.relay = 0
        self.command_pub.publish(command)
        self.get_logger().info(f'Published actuator command: {command}')

        # Wait for 1 second
        time.sleep(1)

        # Publish actuator state = dropped off
        state = ActuatorState()
        state.pickup_state = 3
        self.state_pub.publish(state)
        self.get_logger().info(f'Published actuator state: {state.pickup_state}')

    def handle_request_type_1(self):
        # Command arm directly DOWN, relay ON
        command = ActuatorCommand()
        command.x = 0.0  # Ensure this is a float
        command.y = 0.0  # Ensure this is a float
        command.z = -130.0  # Ensure this is a float
        command.relay = 1
        self.command_pub.publish(command)
        self.get_logger().info(f'Published actuator command: {command}')

        # Wait for 1 second
        time.sleep(1)

        # Check if sensor detect pick up
        if self.sensor_state == 1:
            # Command arm HOME, relay ON
            command.x = 0.0  # Ensure this is a float
            command.y = 0.0  # Ensure this is a float
            command.z = -30.0  # Ensure this is a float
            command.relay = 2
            self.command_pub.publish(command)
            self.get_logger().info(f'Published actuator command: {command}')

            # Publish actuator state = pick up successful
            state = ActuatorState()
            state.pickup_state = 1
            self.state_pub.publish(state)
            self.get_logger().info(f'Published actuator state: {state.pickup_state}')
        else:
            # Wait for 10 seconds
            time.sleep(5)

            # Command arm HOME, relay ON
            command.x = 0.0  # Ensure this is a float
            command.y = 0.0  # Ensure this is a float
            command.z = -30.0  # Ensure this is a float
            command.relay = 1
            self.command_pub.publish(command)
            self.get_logger().info(f'Published actuator command: {command}')

            # Publish actuator state = pick up unsuccessful/unknown
            state = ActuatorState()
            state.pickup_state = 2
            self.state_pub.publish(state)
            self.get_logger().info(f'Published actuator state: {state.pickup_state}')
        

    def handle_request_type_2(self, target_x, target_y):
        # Command arm to TARGET X and Y, relay ON
        command = ActuatorCommand()
        command.x = float(target_x)  # Ensure this is a float
        command.y = float(target_y)  # Ensure this is a float
        command.z = -100.0  # Ensure this is a float
        command.relay = 1
        self.command_pub.publish(command)
        self.get_logger().info(f'Published actuator command: {command}')

        # Wait for 1 second
        time.sleep(1)

        # Check if sensor detect pick up
        if self.sensor_state == 1:
            # Command arm HOME, relay ON
            command.x = 0.0  # Ensure this is a float
            command.y = 0.0  # Ensure this is a float
            command.z = -30.0  # Ensure this is a float
            command.relay = 2
            self.command_pub.publish(command)
            self.get_logger().info(f'Published actuator command: {command}')

            # Publish actuator state = pick up successful
            state = ActuatorState()
            state.pickup_state = 1
            self.state_pub.publish(state)
            self.get_logger().info(f'Published actuator state: {state.pickup_state}')
        else:
            # Wait for 10 seconds
            time.sleep(5)

            # Command arm HOME, relay ON
            command.x = 0.0  # Ensure this is a float
            command.y = 0.0  # Ensure this is a float
            command.z = -30.0  # Ensure this is a float
            command.relay = 1
            self.command_pub.publish(command)
            self.get_logger().info(f'Published actuator command: {command}')

            # Publish actuator state = pick up unsuccessful/unknown
            state = ActuatorState()
            state.pickup_state = 2
            self.state_pub.publish(state)
            self.get_logger().info(f'Published actuator state: {state.pickup_state}')
    
    def handle_request_type_3(self):
        # Command arm to CORNER 1, relay ON
        command = ActuatorCommand()
        command.x = float(90)  # Ensure this is a float
        command.y = float(0)  # Ensure this is a float
        command.z = -100.0  # Ensure this is a float
        command.relay = 1
        self.command_pub.publish(command)
        self.get_logger().info(f'Published actuator command: {command}')

        # Wait for 1 second
        time.sleep(3)
        
        # Command arm to CORNER 2, relay ON
        command = ActuatorCommand()
        command.x = float(0)  # Ensure this is a float
        command.y = float(90)  # Ensure this is a float
        command.z = -100.0  # Ensure this is a float
        command.relay = 1
        self.command_pub.publish(command)
        self.get_logger().info(f'Published actuator command: {command}')

        # Wait for 1 second
        time.sleep(3)

        # Command arm to CORNER 3, relay ON
        command = ActuatorCommand()
        command.x = float(-90)  # Ensure this is a float
        command.y = float(0)  # Ensure this is a float
        command.z = -100.0  # Ensure this is a float
        command.relay = 1
        self.command_pub.publish(command)
        self.get_logger().info(f'Published actuator command: {command}')

        # Wait for 1 second
        time.sleep(3)

        # Command arm to CORNER 4, relay ON
        command = ActuatorCommand()
        command.x = float(0)  # Ensure this is a float
        command.y = float(-90)  # Ensure this is a float
        command.z = -100.0  # Ensure this is a float
        command.relay = 1
        self.command_pub.publish(command)
        self.get_logger().info(f'Published actuator command: {command}')

        # Wait for 1 second
        time.sleep(3)

        command.x = 0.0  # Ensure this is a float
        command.y = 0.0  # Ensure this is a float
        command.z = -30.0  # Ensure this is a float
        command.relay = 1
        self.command_pub.publish(command)
        self.get_logger().info(f'Published actuator command: {command}')

        # Publish actuator state = pick up unsuccessful/unknown
        state = ActuatorState()
        state.pickup_state = 2
        self.state_pub.publish(state)
        self.get_logger().info(f'Published actuator state: {state.pickup_state}')            
    
    def handle_sensor(self, msg):
        # Store the latest sensor state
        self.sensor_state = msg.sensor_value
        self.get_logger().info(f'Received sensor value: {self.sensor_state}')

def main(args=None):
    rclpy.init(args=args)
    actuator_logic = ActuatorLogic()
    rclpy.spin(actuator_logic)
    actuator_logic.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

