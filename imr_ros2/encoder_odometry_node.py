#!/usr/bin/env python3
"""
encoder_odometry_node.py  —  Step 7 (updated from Step 3)
===========================================================
MOCK mode: simulates encoder ticks from /motor_state, then integrates odometry.
REAL mode (later): swap tick simulation for GPIO interrupt counters.
                   Everything else stays identical.

Subscribes:
  /motor_state   Float32MultiArray   [left_signed_speed, right_signed_speed]

Publishes:
  /odom          nav_msgs/Odometry        x, y, theta + covariance
  /odom_path     nav_msgs/Path            live trajectory history (for RViz2)
  /encoder_ticks std_msgs/Int32MultiArray [left_total, right_total]

Note: TF (odom→base_link) is omitted intentionally. nav_msgs/Path displays
in RViz2 work directly from frame_id='odom' matching Fixed Frame — no TF needed.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, PoseStamped


class EncoderOdometryNode(Node):

    # --- Calibration (matches real robot constants) ---
    M_PER_TICK              = 0.014   # meters per encoder pulse
    BASELINE                = 0.31    # wheel separation (meters)
    TIMER_HZ                = 50      # odometry publish rate (Hz)
    TICKS_PER_SEC_PER_UNIT  = 42.0   # tick rate at speed=1.0 (from CSV data)
    STRAIGHT_CMD_EPS        = 0.12   # speed-difference threshold for straight suppression

    def __init__(self):
        super().__init__('encoder_odometry_node')

        # Subscriber: signed motor speeds from mock motor controller
        self._motor_sub = self.create_subscription(
            Float32MultiArray,
            '/motor_state',
            self._motor_state_cb,
            qos_profile=10,
        )

        # Publishers
        self._odom_pub = self.create_publisher(Odometry,        '/odom',          10)
        self._tick_pub = self.create_publisher(Int32MultiArray, '/encoder_ticks', 10)
        self._path_pub = self.create_publisher(Path,            '/odom_path',     10)

        # Accumulated pose history for /odom_path
        self._odom_path = Path()
        self._odom_path.header.frame_id = 'odom'

        # 50 Hz integration timer
        self._timer = self.create_timer(1.0 / self.TIMER_HZ, self._timer_cb)

        # Current motor speeds (updated by subscriber)
        self._left_speed  = 0.0
        self._right_speed = 0.0

        # Sub-integer tick accumulators (avoid losing fractional ticks each cycle)
        self._left_frac  = 0.0
        self._right_frac = 0.0

        # Cumulative integer tick counts
        self._left_ticks  = 0
        self._right_ticks = 0

        # Previous counts (for computing deltas each cycle)
        self._prev_left  = 0
        self._prev_right = 0

        # Robot pose
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0  # radians, +CCW

        self._cycle = 0

        self.get_logger().info('EncoderOdometryNode started  [MOCK mode — no GPIO]')
        self.get_logger().info(
            f'  M_PER_TICK={self.M_PER_TICK} m  |  '
            f'BASELINE={self.BASELINE} m  |  '
            f'tick_rate={self.TICKS_PER_SEC_PER_UNIT} ticks/s @ speed=1.0  |  '
            f'{self.TIMER_HZ} Hz timer'
        )
        self.get_logger().info('  Waiting for /motor_state ...')

    # ------------------------------------------------------------------ #
    #  Subscriber callback                                                 #
    # ------------------------------------------------------------------ #
    def _motor_state_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self._left_speed  = msg.data[0]
            self._right_speed = msg.data[1]

    # ------------------------------------------------------------------ #
    #  50 Hz timer: simulate ticks → integrate odometry → publish         #
    # ------------------------------------------------------------------ #
    def _timer_cb(self):
        self._cycle += 1
        dt = 1.0 / self.TIMER_HZ

        # --- 1. Simulate encoder ticks from motor speed ---
        left_rate  = abs(self._left_speed)  * self.TICKS_PER_SEC_PER_UNIT
        right_rate = abs(self._right_speed) * self.TICKS_PER_SEC_PER_UNIT

        self._left_frac  += left_rate  * dt
        self._right_frac += right_rate * dt

        new_left  = int(self._left_frac)
        new_right = int(self._right_frac)
        self._left_frac  -= new_left
        self._right_frac -= new_right

        self._left_ticks  += new_left
        self._right_ticks += new_right

        # --- 2. Compute tick deltas since last cycle ---
        delta_L = self._left_ticks  - self._prev_left
        delta_R = self._right_ticks - self._prev_right
        self._prev_left  = self._left_ticks
        self._prev_right = self._right_ticks

        # --- 3. Integrate odometry (midpoint rule — same as path_extract.py) ---
        if delta_L > 0 or delta_R > 0:
            sign_L = 1.0 if self._left_speed  >= 0 else -1.0
            sign_R = 1.0 if self._right_speed >= 0 else -1.0

            dL = delta_L * sign_L * self.M_PER_TICK
            dR = delta_R * sign_R * self.M_PER_TICK
            ds = 0.5 * (dL + dR)

            # Straight-segment suppression: if both wheels same direction
            # and speed difference is small → treat as perfectly straight
            same_dir   = (sign_L == sign_R)
            spd_diff   = abs(abs(self._left_speed) - abs(self._right_speed))
            if same_dir and spd_diff < self.STRAIGHT_CMD_EPS:
                dtheta = 0.0
            else:
                dtheta = (dR - dL) / self.BASELINE

            theta_mid   = self._theta + 0.5 * dtheta
            self._x    += ds * math.cos(theta_mid)
            self._y    += ds * math.sin(theta_mid)
            self._theta = self._wrap(self._theta + dtheta)

        # --- 4. Publish /encoder_ticks ---
        tick_msg = Int32MultiArray()
        tick_msg.data = [self._left_ticks, self._right_ticks]
        self._tick_pub.publish(tick_msg)

        # --- 5. Publish /odom ---
        self._publish_odom()

        # --- 6. Log summary every second ---
        if self._cycle % self.TIMER_HZ == 0:
            self.get_logger().info(
                f'ODOM  '
                f'x={self._x:+.4f} m  '
                f'y={self._y:+.4f} m  '
                f'θ={math.degrees(self._theta):+.2f}°  |  '
                f'ticks  L={self._left_ticks}  R={self._right_ticks}'
            )

    # ------------------------------------------------------------------ #
    #  Build and publish nav_msgs/Odometry                                 #
    # ------------------------------------------------------------------ #
    def _publish_odom(self):
        now = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp    = now
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'

        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self._yaw_to_quat(self._theta)

        # Estimated velocities (from current motor command)
        v_fwd = 0.5 * (self._left_speed + self._right_speed) \
                * self.M_PER_TICK * self.TICKS_PER_SEC_PER_UNIT
        v_rot = (self._right_speed - self._left_speed) \
                * self.M_PER_TICK * self.TICKS_PER_SEC_PER_UNIT / self.BASELINE

        msg.twist.twist.linear.x  = v_fwd
        msg.twist.twist.angular.z = v_rot

        # Realistic covariance for pure wheel odometry (not zero — not identity)
        # Indices: [x,y,z,roll,pitch,yaw] → diagonal positions 0,7,14,21,28,35
        msg.pose.covariance[0]  = 0.001   # x
        msg.pose.covariance[7]  = 0.001   # y
        msg.pose.covariance[35] = 0.010   # yaw
        msg.twist.covariance[0]  = 0.001
        msg.twist.covariance[35] = 0.010

        self._odom_pub.publish(msg)

        # --- Append to live trajectory path (publish every 5th cycle to save memory) ---
        if self._cycle % 5 == 0:
            pose = PoseStamped()
            pose.header.stamp    = now
            pose.header.frame_id = 'odom'
            pose.pose.position.x = self._x
            pose.pose.position.y = self._y
            pose.pose.position.z = 0.0
            pose.pose.orientation = self._yaw_to_quat(self._theta)
            self._odom_path.header.stamp = now
            self._odom_path.poses.append(pose)
            self._path_pub.publish(self._odom_path)

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #
    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    @staticmethod
    def _wrap(angle: float) -> float:
        while angle >  math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
