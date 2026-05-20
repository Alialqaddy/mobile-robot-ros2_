#!/usr/bin/env python3
"""
mock_motor_controller_node.py  —  Step 2
=========================================
Subscribes to /cmd_vel (geometry_msgs/Twist).
Converts Twist → differential drive left/right speeds.
Instead of writing to GPIO, prints the result and publishes two topics:

  /motor_state      Float32MultiArray  [left_signed_speed, right_signed_speed]
  /motor_direction  String             FORWARD | BACKWARD | SPIN_LEFT | SPIN_RIGHT | STOP

The sign in /motor_state tells the encoder node (Step 3) how to sign tick deltas.

No hardware. No GPIO. Safe to run on any machine.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String


class MockMotorControllerNode(Node):

    # Mirror the real robot calibration from robot_params (to be loaded as params later)
    BASE_MIN = 0.25   # below this absolute value → motor does not overcome stiction
    BASE_MAX = 1.00   # clamp ceiling

    def __init__(self):
        super().__init__('mock_motor_controller_node')

        # --- Subscribers ---
        self._cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_cb,
            qos_profile=10,
        )

        # --- Publishers ---
        self._state_pub = self.create_publisher(
            Float32MultiArray,
            '/motor_state',
            qos_profile=10,
        )
        self._dir_pub = self.create_publisher(
            String,
            '/motor_direction',
            qos_profile=10,
        )

        self._last_direction = ''

        self.get_logger().info('MockMotorControllerNode started')
        self.get_logger().info('  Subscribing : /cmd_vel')
        self.get_logger().info('  Publishing  : /motor_state, /motor_direction')
        self.get_logger().info('  [NO GPIO]  PWM values printed to terminal only')

    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist):
        linear_x  = msg.linear.x
        angular_z = msg.angular.z

        # Differential drive kinematics
        left_speed  = linear_x - angular_z
        right_speed = linear_x + angular_z

        # Clamp to [-BASE_MAX, +BASE_MAX]
        left_speed  = max(-self.BASE_MAX, min(self.BASE_MAX, left_speed))
        right_speed = max(-self.BASE_MAX, min(self.BASE_MAX, right_speed))

        # Apply stiction deadband — below BASE_MIN the real motor stalls
        if abs(left_speed)  < self.BASE_MIN:
            left_speed  = 0.0
        if abs(right_speed) < self.BASE_MIN:
            right_speed = 0.0

        direction = self._classify(left_speed, right_speed)

        # Log only on direction change to avoid flooding the terminal
        if direction != self._last_direction:
            self._last_direction = direction
            self.get_logger().info(
                f'[CMD_VEL] linear.x={linear_x:+.3f}  angular.z={angular_z:+.3f}  →  '
                f'{direction}  |  '
                f'left={left_speed:+.3f}  right={right_speed:+.3f}'
            )

        # Publish signed speeds — encoder node uses the sign for tick direction
        state_msg = Float32MultiArray()
        state_msg.data = [float(left_speed), float(right_speed)]
        self._state_pub.publish(state_msg)

        # Publish human-readable direction string
        dir_msg = String()
        dir_msg.data = direction
        self._dir_pub.publish(dir_msg)

    # ------------------------------------------------------------------
    @staticmethod
    def _classify(left: float, right: float) -> str:
        dead = 0.01  # treat anything below this as zero for classification
        l_zero = abs(left)  < dead
        r_zero = abs(right) < dead

        if l_zero and r_zero:
            return 'STOP'
        if left > dead and right > dead:
            return 'FORWARD'
        if left < -dead and right < -dead:
            return 'BACKWARD'
        if left < -dead and right > dead:
            return 'SPIN_LEFT'
        if left > dead and right < -dead:
            return 'SPIN_RIGHT'
        # One wheel zero, one non-zero → pivot
        if l_zero and right > dead:
            return 'PIVOT_RIGHT'
        if r_zero and left > dead:
            return 'PIVOT_LEFT'
        return 'STOP'


def main(args=None):
    rclpy.init(args=args)
    node = MockMotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
