#!/usr/bin/env python3
# =============================================================================
# teleop_b3rb.py
# -----------------------------------------------------------------------------
# NXP Cup India 2026 - manual keyboard driving for testing. NOT part of the
# submission (keep it outside b3rb_ros_line_follower/).
#
# PURPOSE
#   Lets you drive the B3RB by hand so you can park it in front of a QR sign
#   board, measure decode range, check zone distances, and generally position
#   the buggy without waiting for the autonomous controller to wander there.
#
# HOW IT WORKS
#   Publishes sensor_msgs/Joy to /cerebri/in/joy at 20 Hz, exactly like the
#   runner does:
#       buttons = [1,0,0,0,0,0,0,1]   -> request MANUAL mode + ARM
#       axes    = [0, thrust, 0, turn]
#   Cerebri needs mode set before arming, which is why the same button pattern
#   is sent continuously rather than once.
#
# IMPORTANT
#   Do NOT run this at the same time as the `runner` node - both publish to
#   /cerebri/in/joy and the buggy will receive interleaved conflicting commands.
#
# CONTROLS
#   w / s : increase / decrease forward thrust
#   a / d : steer left / right
#   space : all stop (thrust and steering to zero)
#   x     : zero the steering only
#   q     : quit (sends a stop first)
#
# USAGE
#   cogniws
#   python3 teleop_b3rb.py
# =============================================================================

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

SPEED_STEP = 0.05
TURN_STEP = 0.10
SPEED_LIMIT = 0.60          # keep it gentle - this is for positioning, not racing
TURN_LIMIT = 1.00
PUBLISH_HZ = 20.0

HELP = """
B3RB keyboard teleop
--------------------
  w / s   thrust  +/-
  a / d   steer   left/right
  space   full stop
  x       centre steering
  q       quit

(do not run the `runner` node at the same time)
"""


class Teleop(Node):

    def __init__(self):
        super().__init__('teleop_b3rb')
        self.pub = self.create_publisher(Joy, '/cerebri/in/joy', 10)
        self.speed = 0.0
        self.turn = 0.0
        self.create_timer(1.0 / PUBLISH_HZ, self.tick)

    def tick(self):
        msg = Joy()
        # Same button pattern the runner uses: request manual mode and arm.
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg.axes = [0.0, float(self.speed), 0.0, float(self.turn)]
        self.pub.publish(msg)

    def apply_key(self, key):
        if key == 'w':
            self.speed = min(self.speed + SPEED_STEP, SPEED_LIMIT)
        elif key == 's':
            self.speed = max(self.speed - SPEED_STEP, -SPEED_LIMIT)
        elif key == 'a':
            self.turn = min(self.turn + TURN_STEP, TURN_LIMIT)
        elif key == 'd':
            self.turn = max(self.turn - TURN_STEP, -TURN_LIMIT)
        elif key == 'x':
            self.turn = 0.0
        elif key == ' ':
            self.speed = 0.0
            self.turn = 0.0
        return f"thrust={self.speed:+.2f}  steer={self.turn:+.2f}   "


def read_key(timeout=0.05):
    """Non-blocking single-character read from stdin."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.read(1)
    return None


def main():
    rclpy.init()
    node = Teleop()

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key()
            if key:
                if key == 'q':
                    break
                status = node.apply_key(key.lower())
                sys.stdout.write("\r" + status)
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the buggy stopped rather than coasting away.
        node.speed = 0.0
        node.turn = 0.0
        for _ in range(10):
            node.tick()
            rclpy.spin_once(node, timeout_sec=0.01)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\nteleop stopped.")


if __name__ == '__main__':
    main()
