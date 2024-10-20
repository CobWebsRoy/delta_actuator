import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from actuator_msgs.msg import ActuatorCommand
from math import atan2, sqrt, cos, sin, pi

class DeltaKinematics:
    def __init__(self, ArmLength, RodLength, BassTri, PlatformTri):
        self.ArmLength = ArmLength
        self.RodLength = RodLength
        self.BassTri = BassTri
        self.PlatformTri = PlatformTri
        self.tan30 = sqrt(3) / 3.0
        self.sin120 = sqrt(3) / 2.0
        self.cos120 = -0.5
        self.tan60 = sqrt(3)
        self.sin30 = 0.5
        self.pi = pi
        self.x = self.y = self.z = 0.0
        self.a = self.b = self.c = 0.0

    def inverse_kinematics(self, x0, y0, z0):
        a, err = self.delta_calcAngleYZ(x0, y0, z0)
        b, err = self.delta_calcAngleYZ(x0 * self.cos120 + y0 * self.sin120, y0 * self.cos120 - x0 * self.sin120, z0)
        c, err = self.delta_calcAngleYZ(x0 * self.cos120 - y0 * self.sin120, y0 * self.cos120 + x0 * self.sin120, z0)
        return a, b, c

    def delta_calcAngleYZ(self, x0, y0, z0):
        y1 = -0.5 * self.tan30 * self.BassTri
        y0 -= 0.5 * self.tan30 * self.PlatformTri
        aV = (x0 ** 2 + y0 ** 2 + z0 ** 2 + self.ArmLength ** 2 - self.RodLength ** 2 - y1 ** 2) / (2.0 * z0)
        bV = (y1 - y0) / z0
        dV = -(aV + bV * y1) ** 2 + self.ArmLength * (bV ** 2 * self.ArmLength + self.ArmLength)
        if dV < 0:
            return None, "Non-existing point"
        yj = (y1 - aV * bV - sqrt(dV)) / (bV ** 2 + 1)
        zj = aV + bV * yj
        return atan2(-zj, (y1 - yj)) * 180.0 / pi, None


def main(args=None):
    rclpy.init(args=args)
    
    # Initialize ServoKit
    kit = ServoKit(channels=16)
    
    # Initialize servos to 0 degrees
    kit.servo[0].angle = 0
    kit.servo[1].angle = 0
    kit.servo[2].angle = 0

    # Create a ROS 2 node
    node = Node('servo_controller')
    
    # Initialize delta robot kinematics
    delta_kinematics = DeltaKinematics(ArmLength=100, RodLength=100, BassTri=30, PlatformTri=30)

    def command_callback(msg):
        # Extract target x, y, z from the ActuatorCommand message
        target_x = msg.x
        target_y = msg.y
        target_z = msg.z

        node.get_logger().info(f'Received command: x={target_x}, y={target_y}, z={target_z}')

        # Perform inverse kinematics to get the corresponding angles
        theta_a, theta_b, theta_c = delta_kinematics.inverse_kinematics(target_x, target_y, target_z)

        if theta_a is not None and theta_b is not None and theta_c is not None:
            # Move the servos to the calculated angles
            kit.servo[0].angle = theta_a
            kit.servo[1].angle = theta_b
            kit.servo[2].angle = theta_c
            node.get_logger().info(f'Moving servos to angles: A={theta_a}, B={theta_b}, C={theta_c}')
        else:
            node.get_logger().error('Invalid command, non-existing point in inverse kinematics.')

    # Create a subscription to ActuatorCommand messages
    subscription = node.create_subscription(
        ActuatorCommand,
        '/actuator_commands',
        command_callback,
        10
    )

    # Spin the node so that it continuously listens for messages
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

