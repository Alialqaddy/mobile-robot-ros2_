#!/usr/bin/env python3
"""
waypoint_follower_node.py  —  Step 5 (updated from Step 4)
============================================================
Timer-based state machine: IDLE → TURNING → DRIVING → DONE

Subscribes:
  /odom              nav_msgs/Odometry     current robot pose

Publishes:
  /cmd_vel           geometry_msgs/Twist   drive commands
  /follower/status   std_msgs/String       state machine status

Services:
  /follower/start    std_srvs/Trigger      start (uses path_file param or built-in)
  /follower/abort    std_srvs/Trigger      stop immediately

Parameters:
  path_file  string  default=''   path to a CSV file with x,y columns
                                  if empty, built-in 0.5m square is used

Usage with real recorded path:
  ros2 run imr_ros2 waypoint_follower_node \
    --ros-args -p path_file:=/home/ali/imr_ws/paths/last_path.csv
"""

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import String
from std_srvs.srv import Trigger


class WaypointFollowerNode(Node):

    # --- Motion parameters (match real robot calibration) ---
    DRIVE_SPEED      = 0.30   # forward PWM (well above BASE_MIN=0.25)
    SLOW_SPEED       = 0.25   # approach speed (at BASE_MIN boundary)
    TURN_SPEED       = 0.35   # in-place spin speed
    ANGLE_MARGIN     = math.radians(5.0)   # 5° heading tolerance
    APPROACH_ZONE    = 0.15   # slow down within 15 cm of waypoint
    REACHED_DIST     = 0.05   # waypoint considered reached within 5 cm
    TIMER_HZ         = 20     # state machine update rate (Hz)

    # --- Built-in test path: 0.5 m square ---
    TEST_WAYPOINTS = [
        (0.00, 0.00),   # 0: start position (robot begins here)
        (0.50, 0.00),   # 1: 0.5 m forward along +X
        (0.50, 0.50),   # 2: 0.5 m along +Y (requires 90° left turn)
        (0.00, 0.50),   # 3: 0.5 m along -X (requires 90° left turn)
        (0.00, 0.00),   # 4: back to start  (requires 90° left turn)
    ]

    # --- State names ---
    IDLE    = 'IDLE'
    TURNING = 'TURNING'
    DRIVING = 'DRIVING'
    DONE    = 'DONE'

    def __init__(self):
        super().__init__('waypoint_follower_node')

        # Publishers
        self._cmd_pub    = self.create_publisher(Twist,    '/cmd_vel',         10)
        self._status_pub = self.create_publisher(String,   '/follower/status', 10)
        # Latched QoS: late subscribers (like RViz2) receive the last published message
        _latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._plan_pub   = self.create_publisher(NavPath, '/planned_path', _latched)

        # Subscriber: get robot pose from odometry
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        # Subscriber: obstacle alert from ultrasonic node
        self._obstacle_sub = self.create_subscription(
            String, '/obstacle_alert', self._obstacle_cb, 10)

        # Obstacle state (set by callback, read by state machine timer)
        self._obstacle_alert = 'CLEAR'   # "CLEAR" | "SLOW" | "STOP"

        # Services
        self._start_srv = self.create_service(
            Trigger, '/follower/start', self._start_cb)
        self._abort_srv = self.create_service(
            Trigger, '/follower/abort', self._abort_cb)

        # 20 Hz state machine timer
        self._timer = self.create_timer(1.0 / self.TIMER_HZ, self._tick)

        # Current robot pose (written by odom callback, read by timer)
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0   # radians, +CCW

        # Path tracking
        self._state           = self.IDLE
        self._waypoints       = []
        self._wp_idx          = 0     # index of current TARGET waypoint
        self._desired_heading = 0.0   # heading to current target

        # ROS2 parameter: path to CSV file (empty = use built-in square)
        self.declare_parameter('path_file', '')

        path_file = self.get_parameter('path_file').get_parameter_value().string_value
        if path_file:
            self.get_logger().info(f'  path_file parameter set: {path_file}')
        else:
            self.get_logger().info('  path_file not set — will use built-in 0.5m square')

        self.get_logger().info('WaypointFollowerNode started')
        self.get_logger().info('  State: IDLE')
        self.get_logger().info(
            '  To start: ros2 service call /follower/start std_srvs/srv/Trigger "{}"'
        )

    # ================================================================== #
    #  Subscriber callbacks                                                #
    # ================================================================== #
    def _obstacle_cb(self, msg: String):
        new_alert = msg.data
        if new_alert != self._obstacle_alert:
            if new_alert == 'STOP':
                self.get_logger().warn(
                    '[OBSTACLE] STOP received — pausing forward motion'
                )
            elif new_alert == 'SLOW':
                self.get_logger().warn(
                    '[OBSTACLE] SLOW received — reducing speed'
                )
            elif new_alert == 'CLEAR' and self._obstacle_alert != 'CLEAR':
                self.get_logger().info(
                    '[OBSTACLE] CLEAR — resuming normal speed'
                )
            self._obstacle_alert = new_alert

    def _odom_cb(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self._theta = 2.0 * math.atan2(qz, qw)

    # ================================================================== #
    #  Service callbacks                                                   #
    # ================================================================== #
    def _start_cb(self, request, response):
        if self._state != self.IDLE:
            response.success = False
            response.message = f'Already running in state={self._state}. Call /follower/abort first.'
            self.get_logger().warn(response.message)
            return response

        path_file = self.get_parameter('path_file').get_parameter_value().string_value

        if path_file:
            loaded = self._load_csv(path_file)
            if loaded is None:
                response.success = False
                response.message = f'Failed to load CSV: {path_file}'
                self.get_logger().error(response.message)
                return response
            self._waypoints = loaded
            source = f'CSV: {Path(path_file).name}'
        else:
            self._waypoints = list(self.TEST_WAYPOINTS)
            source = 'built-in 0.5m square'

        self._wp_idx = 1   # waypoint[0] is the start; first target is waypoint[1]

        n_segs = len(self._waypoints) - 1
        response.success = True
        response.message = (
            f'Loaded {len(self._waypoints)} waypoints ({n_segs} segments) from {source}. Starting...'
        )
        self.get_logger().info(f'[START] {response.message}')
        self.get_logger().info(
            f'  First WP: ({self._waypoints[1][0]:.3f}, {self._waypoints[1][1]:.3f})  '
            f'Last WP: ({self._waypoints[-1][0]:.3f}, {self._waypoints[-1][1]:.3f})'
        )
        self._publish_planned_path()

        # Compute first desired heading and enter TURNING
        self._compute_desired_heading()
        heading_err = self._wrap(self._desired_heading - self._theta)

        if abs(heading_err) > self.ANGLE_MARGIN:
            self._set_state(self.TURNING)
            self.get_logger().info(
                f'  First turn needed: {math.degrees(heading_err):.1f}°'
            )
        else:
            self._set_state(self.DRIVING)
            self.get_logger().info('  Heading already aligned — driving immediately')

        return response

    def _abort_cb(self, request, response):
        self._publish_stop()
        prev = self._state
        self._set_state(self.IDLE)
        response.success = True
        response.message = f'Aborted from state={prev}. Robot stopped.'
        self.get_logger().warn(f'[ABORT] {response.message}')
        return response

    # ================================================================== #
    #  20 Hz state machine                                                 #
    # ================================================================== #
    def _tick(self):
        if self._state in (self.IDLE, self.DONE):
            return

        if self._state == self.TURNING:
            self._do_turning()

        elif self._state == self.DRIVING:
            self._do_driving()

        # Publish status string for monitoring
        target = self._waypoints[self._wp_idx] if self._wp_idx < len(self._waypoints) else ('--', '--')
        status = String()
        status.data = (
            f'{self._state}  '
            f'wp={self._wp_idx}/{len(self._waypoints)-1}  '
            f'robot=({self._x:.3f},{self._y:.3f})  '
            f'θ={math.degrees(self._theta):.1f}°  '
            f'target=({target[0]:.2f},{target[1]:.2f})'
            if isinstance(target[0], float) else
            f'{self._state}  wp=DONE'
        )
        self._status_pub.publish(status)

    # ------------------------------------------------------------------ #
    def _do_turning(self):
        self._compute_desired_heading()
        heading_error = self._wrap(self._desired_heading - self._theta)

        if abs(heading_error) <= self.ANGLE_MARGIN:
            self._publish_stop()
            self.get_logger().info(
                f'[ALIGNED] wp={self._wp_idx}  '
                f'residual error={math.degrees(heading_error):.2f}°  '
                f'→ DRIVING'
            )
            self._set_state(self.DRIVING)
            return

        # Proportional turn: use fixed TURN_SPEED with correct sign
        cmd = Twist()
        cmd.angular.z = math.copysign(self.TURN_SPEED, heading_error)
        self._cmd_pub.publish(cmd)

    # ------------------------------------------------------------------ #
    def _do_driving(self):
        tx, ty = self._waypoints[self._wp_idx]
        dx   = tx - self._x
        dy   = ty - self._y
        dist = math.hypot(dx, dy)

        if dist <= self.REACHED_DIST:
            self._publish_stop()
            self.get_logger().info(
                f'[REACHED] wp={self._wp_idx} ({tx:.2f},{ty:.2f})  '
                f'position error={dist*100:.1f} cm'
            )
            self._advance_to_next_waypoint()
            return

        # Obstacle check — evaluated every timer tick, non-blocking
        if self._obstacle_alert == 'STOP':
            self._publish_stop()
            return   # stay in DRIVING state; resume next tick when CLEAR arrives

        # Speed: slow near waypoint OR when SLOW obstacle alert
        if self._obstacle_alert == 'SLOW' or dist <= self.APPROACH_ZONE:
            speed = self.SLOW_SPEED
        else:
            speed = self.DRIVE_SPEED

        cmd = Twist()
        cmd.linear.x = speed
        self._cmd_pub.publish(cmd)

    # ------------------------------------------------------------------ #
    def _advance_to_next_waypoint(self):
        self._wp_idx += 1

        if self._wp_idx >= len(self._waypoints):
            self._set_state(self.DONE)
            last_x, last_y = self._waypoints[-1]
            pos_err = math.hypot(self._x - last_x, self._y - last_y)
            self.get_logger().info('=' * 55)
            self.get_logger().info('[DONE] All waypoints reached!')
            self.get_logger().info(
                f'  Final position : ({self._x:.4f}, {self._y:.4f})  '
                f'θ={math.degrees(self._theta):.1f}°'
            )
            self.get_logger().info(
                f'  Expected end   : ({last_x:.4f}, {last_y:.4f})'
            )
            self.get_logger().info(
                f'  Position error : {pos_err * 100:.1f} cm'
                + ('  ✓ excellent' if pos_err < 0.06 else
                   '  ✓ good'      if pos_err < 0.12 else
                   '  ⚠ check calibration')
            )
            self.get_logger().info('=' * 55)
            return

        # Compute heading to next waypoint
        self._compute_desired_heading()
        heading_error = self._wrap(self._desired_heading - self._theta)

        if abs(heading_error) > self.ANGLE_MARGIN:
            self.get_logger().info(
                f'[NEXT] wp={self._wp_idx}  '
                f'need turn of {math.degrees(heading_error):.1f}°  → TURNING'
            )
            self._set_state(self.TURNING)
        else:
            self.get_logger().info(
                f'[NEXT] wp={self._wp_idx}  heading ok  → DRIVING'
            )
            self._set_state(self.DRIVING)

    # ================================================================== #
    #  Helpers                                                             #
    # ================================================================== #
    def _publish_planned_path(self):
        """Publish all waypoints as a nav_msgs/Path for RViz2 display."""
        path_msg = NavPath()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        for x, y in self._waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self._plan_pub.publish(path_msg)

    def _load_csv(self, path_file: str):
        """Load x,y waypoints from a CSV file. Returns list of (x,y) tuples or None on error."""
        p = Path(path_file)
        if not p.exists():
            self.get_logger().error(f'CSV file not found: {path_file}')
            return None
        waypoints = []
        try:
            with open(p, newline='', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    x = float(row['x'])
                    y = float(row['y'])
                    waypoints.append((x, y))
        except Exception as e:
            self.get_logger().error(f'Error reading CSV: {e}')
            return None
        if len(waypoints) < 2:
            self.get_logger().error('CSV has fewer than 2 waypoints — nothing to follow')
            return None
        self.get_logger().info(f'  Read {len(waypoints)} waypoints from {p.name}')
        return waypoints

    def _compute_desired_heading(self):
        if self._wp_idx >= len(self._waypoints):
            return
        tx, ty = self._waypoints[self._wp_idx]
        self._desired_heading = math.atan2(ty - self._y, tx - self._x)

    def _publish_stop(self):
        self._cmd_pub.publish(Twist())

    def _set_state(self, new_state: str):
        if new_state != self._state:
            self.get_logger().info(f'  State: {self._state} → {new_state}')
            self._state = new_state

    @staticmethod
    def _wrap(angle: float) -> float:
        while angle >  math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
