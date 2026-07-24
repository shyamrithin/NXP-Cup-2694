#!/usr/bin/env python3
# =============================================================================
# qr_range_test.py
# -----------------------------------------------------------------------------
# NXP Cup India 2026 - measurement tool. NOT part of the submission.
#
# PURPOSE
#   Answers one question: at what distance does the QR detector actually decode
#   a sign board? That number decides ZONE_WALL_DIST in the runner, which gates
#   every server transmission - and transmitting outside a zone is a scored
#   penalty, so guessing it is expensive.
#
# WHAT IT DOES
#   Subscribes to /scan and /qr_detection simultaneously and prints a live
#   status line showing the current forward distance. Every time a NEW decode
#   arrives it records the distance at that instant, so you get a running table
#   of "first decode happened at X metres".
#
#   The forward bearing is computed from angle_min/angle_increment rather than
#   assuming index 0 faces forward (this scan spans -pi..+pi, so index 0 is
#   actually the REAR - a mistake worth not repeating).
#
# HOW TO USE
#   Run alongside the sim, the real qr_detect node, and teleop:
#     T1  ros2 launch b3rb_gz_bringup sil.launch.py world:=Raceway_1
#     T2  ros2 run b3rb_ros_line_follower qr_detect
#     T3  python3 teleop_b3rb.py
#     T4  python3 qr_range_test.py       <-- this
#
#   Drive slowly toward a QR board and watch the FIRST DECODE line. Then back
#   away until decoding stops to find the drop-out range. Repeat at an angle to
#   see how much off-axis tolerance you have.
#
#   Ctrl-C prints a summary of every decode range recorded.
# =============================================================================

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

FRONT_ARC_DEG = 12          # narrow: we want the distance to what we are facing
STALE_AFTER_SEC = 1.0       # a decode older than this counts as "lost"


class QrRangeTest(Node):

    def __init__(self):
        super().__init__('qr_range_test')
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(String, '/qr_detection', self.qr_cb, 10)

        self.front = float('inf')
        self.last_payload = None
        self.last_decode_time = None
        self.records = []       # (payload, distance) for each new decode

        self.create_timer(0.25, self.report)
        self.get_logger().info(
            "QR range test running. Drive slowly toward a QR board.")

    def scan_cb(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return
        inc = msg.angle_increment or (2.0 * math.pi / n)

        def idx(bearing):
            return max(0, min(n - 1, int(round((bearing - msg.angle_min) / inc))))

        lo = idx(math.radians(-FRONT_ARC_DEG))
        hi = idx(math.radians(FRONT_ARC_DEG))
        if lo > hi:
            lo, hi = hi, lo
        valid = [r for r in msg.ranges[lo:hi + 1]
                 if r is not None and not math.isinf(r)
                 and not math.isnan(r) and r > 0.05]
        self.front = min(valid) if valid else float('inf')

    def qr_cb(self, msg):
        payload = (msg.data or "").strip()
        if not payload:
            return
        now = self.get_clock().now().nanoseconds / 1e9

        is_new = (payload != self.last_payload
                  or self.last_decode_time is None
                  or (now - self.last_decode_time) > STALE_AFTER_SEC)

        if is_new:
            d = self.front
            self.records.append((payload, d))
            dist = f"{d:.2f} m" if not math.isinf(d) else "no wall in front"
            self.get_logger().info(f"*** FIRST DECODE  '{payload}'  at {dist}")

        self.last_payload = payload
        self.last_decode_time = now

    def report(self):
        now = self.get_clock().now().nanoseconds / 1e9
        fresh = (self.last_decode_time is not None
                 and (now - self.last_decode_time) < 0.5)
        front = f"{self.front:5.2f}" if not math.isinf(self.front) else "  inf"
        state = f"DECODING '{self.last_payload}'" if fresh else "no decode"
        print(f"\r front={front} m   {state:<34}", end="", flush=True)

    def summary(self):
        print("\n\n--- decode ranges recorded ---")
        if not self.records:
            print("  (none)")
            return
        for payload, d in self.records:
            dist = f"{d:.2f} m" if not math.isinf(d) else "inf"
            print(f"  {payload:<24} {dist}")
        finite = [d for _, d in self.records if not math.isinf(d)]
        if finite:
            print(f"\n  max decode distance : {max(finite):.2f} m")
            print(f"  min decode distance : {min(finite):.2f} m")
            print("\n  -> set ZONE_WALL_DIST comfortably BELOW the max, so the "
                  "gate\n     only opens where decoding is reliable.")


def main():
    rclpy.init()
    node = QrRangeTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
