#!/usr/bin/env python3
"""
mock_ultrasonic_node.py  —  Step 6
=====================================
Simulates the HC-SR04 ultrasonic distance sensor without hardware.

Publishes:
  /ultrasonic/range   sensor_msgs/Range    distance in meters (20 Hz)
  /obstacle_alert     std_msgs/String      "CLEAR" | "SLOW" | "STOP"

Parameters:
  mode          string   "always_clear"  — never blocks (default, test normal driving)
                         "inject_slow"   — publish SLOW after inject_after_sec seconds
                         "inject_stop"   — publish STOP after inject_after_sec seconds
  inject_after_sec  float  5.0   seconds after start before injecting obstacle
  clear_after_sec   float  3.0   seconds after obstacle before clearing

Thresholds (match real robot config):
  STOP threshold : 0.30 m
  SLOW threshold : 0.40 m
  CLEAR          : 1.00 m (free)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range


class MockUltrasonicNode(Node):

    STOP_DIST  = 0.25   # meters — publish this when injecting STOP
    SLOW_DIST  = 0.35   # meters — publish this when injecting SLOW
    CLEAR_DIST = 1.00   # meters — free path
    TIMER_HZ   = 20

    def __init__(self):
        super().__init__('mock_ultrasonic_node')

        # Parameters
        self.declare_parameter('mode',             'always_clear')
        self.declare_parameter('inject_after_sec', 5.0)
        self.declare_parameter('clear_after_sec',  3.0)

        self._mode             = self.get_parameter('mode').get_parameter_value().string_value
        self._inject_after     = self.get_parameter('inject_after_sec').get_parameter_value().double_value
        self._clear_after      = self.get_parameter('clear_after_sec').get_parameter_value().double_value

        # Publishers
        self._range_pub = self.create_publisher(Range,  '/ultrasonic/range',  10)
        self._alert_pub = self.create_publisher(String, '/obstacle_alert',    10)

        # 20 Hz timer
        self._timer = self.create_timer(1.0 / self.TIMER_HZ, self._timer_cb)

        self._elapsed      = 0.0
        self._dt           = 1.0 / self.TIMER_HZ
        self._obstacle_on  = False
        self._obstacle_cleared = False
        self._last_alert   = ''

        self.get_logger().info(f'MockUltrasonicNode started  mode={self._mode}')
        if self._mode in ('inject_slow', 'inject_stop'):
            self.get_logger().info(
                f'  Will inject {self._mode.split("_")[1].upper()} '
                f'at t={self._inject_after:.1f}s  '
                f'clear at t={self._inject_after + self._clear_after:.1f}s'
            )
        self.get_logger().info(
            '  Publishing /ultrasonic/range and /obstacle_alert at 20 Hz'
        )

    def _timer_cb(self):
        self._elapsed += self._dt

        # Determine current distance based on mode and time
        if self._mode == 'always_clear':
            dist  = self.CLEAR_DIST
            alert = 'CLEAR'

        elif self._mode == 'inject_slow':
            if (self._inject_after <= self._elapsed
                    < self._inject_after + self._clear_after):
                dist  = self.SLOW_DIST
                alert = 'SLOW'
            else:
                dist  = self.CLEAR_DIST
                alert = 'CLEAR'

        elif self._mode == 'inject_stop':
            if (self._inject_after <= self._elapsed
                    < self._inject_after + self._clear_after):
                dist  = self.STOP_DIST
                alert = 'STOP'
            else:
                dist  = self.CLEAR_DIST
                alert = 'CLEAR'
        else:
            dist  = self.CLEAR_DIST
            alert = 'CLEAR'

        # Log only on alert transitions (not every tick)
        if alert != self._last_alert:
            if alert == 'STOP':
                self.get_logger().warn(
                    f'[OBSTACLE] STOP injected at t={self._elapsed:.1f}s  '
                    f'dist={dist:.2f}m  — follower should pause'
                )
            elif alert == 'SLOW':
                self.get_logger().warn(
                    f'[OBSTACLE] SLOW injected at t={self._elapsed:.1f}s  '
                    f'dist={dist:.2f}m  — follower should slow down'
                )
            elif alert == 'CLEAR' and self._last_alert != '':
                self.get_logger().info(
                    f'[OBSTACLE] CLEAR at t={self._elapsed:.1f}s  '
                    f'dist={dist:.2f}m  — follower should resume'
                )
            self._last_alert = alert

        # Publish Range message
        range_msg = Range()
        range_msg.header.stamp    = self.get_clock().now().to_msg()
        range_msg.header.frame_id = 'ultrasonic_frame'
        range_msg.radiation_type  = Range.ULTRASOUND
        range_msg.field_of_view   = 0.26   # ~15 degrees
        range_msg.min_range       = 0.02
        range_msg.max_range       = 1.50
        range_msg.range           = float(dist)
        self._range_pub.publish(range_msg)

        # Publish alert string
        alert_msg = String()
        alert_msg.data = alert
        self._alert_pub.publish(alert_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockUltrasonicNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
