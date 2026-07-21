# =============================================================================
# b3rb_ros_line_follower.py
# -----------------------------------------------------------------------------
# NXP Cup India 2026 - Autonomous Medical Response
# Team: CEM / SRM  |  Node: "runner"
#
# PURPOSE
#   Central mission controller for the B3RB buggy. Owns the mission state
#   machine, the Municipality Server protocol (uid/ack handshake), lane
#   following, and reactive obstacle avoidance.
#
# ARCHITECTURE
#   Perception nodes publish -> this node decides -> Joy command to cerebri.
#
#     /edge_vectors          (EdgeVectors)          <- vectors node   [WORKING]
#     /scan                  (LaserScan)            <- gazebo lidar   [WORKING]
#     /qr_detection          (String)               <- qr_detect node [WORKING]
#     /sign_board_detection  (String)               <- detect node    [STUB]
#     /ServerCommunication   (ServerCommunication)  <- municipality server
#                 |
#                 v
#     /cerebri/in/joy        (Joy)  axes=[0, speed, 0, turn]
#
# MISSION FLOW
#   SEEK_PATIENT -> AT_BUILDING -> WAIT_ASSIGNMENT -> SEEK_HOSPITAL
#     -> AT_BUILDING -> WAIT_NEXT -> (repeat x3) -> SEEK_EXIT -> PARKING -> DONE
#
# DESIGN NOTES
#   1. Routing is a PLUGGABLE input. If sign classification is unavailable the
#      state machine falls back to systematic exploration, still scanning every
#      QR it passes. This guarantees points even if the classifier never lands.
#   2. Zones are invisible. "Inside zone" is inferred from LiDAR proximity to a
#      building wall AND a fresh QR decode. Transmitting outside a zone is a
#      scored penalty, so this gate is deliberately conservative.
#   3. The docs are inconsistent about server payloads: README shows QR text
#      like "{LOC: PATIENT_1}" while ServerCommunicationGuide shows single
#      letters ("A", "X"). We normalise both directions - see CODE_TO_NAME.
#
# TUNING
#   All tunables are grouped in the CONFIG block below.
# =============================================================================

import math
import re
import time
from enum import Enum

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String
from synapse_msgs.msg import EdgeVectors, ServerCommunication

# =============================================================================
# CONFIG
# =============================================================================

QOS = 10

# --- control bounds (fixed by the platform) ---
SPEED_MAX = 1.0
TURN_MAX = 1.0

# --- speed schedule ---
SPEED_CRUISE = 0.55          # straight-line cruise
SPEED_CORNER = 0.30          # when steering hard
SPEED_APPROACH = 0.18        # closing on a building to scan
SPEED_STOP = 0.0
TURN_SLOWDOWN_GAIN = 0.6     # how much |turn| bleeds speed

# --- lane following ---
LANE_KP = 1.0                # proportional gain on normalised deviation
SINGLE_LINE_PUSH = 0.55      # how hard to steer away from a lone boundary
LANE_LOST_TURN = 0.25        # gentle arc while searching for the lane

# --- obstacle avoidance ---
OBSTACLE_STOP_DIST = 0.55    # metres, emergency
OBSTACLE_SLOW_DIST = 1.20    # metres, begin evasive steer
FRONT_ARC_DEG = 40           # +/- degrees treated as "front"
SIDE_ARC_DEG = 70            # sector used to pick a dodge direction

# --- zone estimation (penalty-critical: keep conservative) ---
ZONE_WALL_DIST = 1.60        # building considered "reached" within this range
QR_FRESH_SEC = 1.5           # a QR read older than this is not trusted

# --- server protocol ---
ACK_TIMEOUT_SEC = 1.0        # wait before resending an unacked message
MAX_RETRIES = 4              # our own retry budget (server allows 5)
SRC_BUGGY = 1
DEST_SERVER = 2
ID_BUGGY = 1

# --- mission ---
TOTAL_PATIENTS = 3
PARK_ANNOUNCE_SEC = 45.0     # send PARKED well inside the 60s window

# --- payload normalisation ---
CODE_TO_NAME = {
    "A": "PATIENT_1", "B": "PATIENT_2", "C": "PATIENT_3",
    "X": "HOSPITAL_1", "Y": "HOSPITAL_2", "Z": "HOSPITAL_3",
}
NAME_TO_CODE = {v: k for k, v in CODE_TO_NAME.items()}

BUILDING_RE = re.compile(r"(FAKE_HOSPITAL_\d|HOSPITAL_\d|PATIENT_\d)", re.I)


# =============================================================================
# MISSION STATES
# =============================================================================

class State(Enum):
    INIT = 0
    SEEK_PATIENT = 1        # driving, looking for the target patient building
    AT_BUILDING = 2         # stopped at a building, inside zone, transmitting
    WAIT_ASSIGNMENT = 3     # sent patient id, waiting for hospital
    SEEK_HOSPITAL = 4       # driving to assigned hospital
    WAIT_NEXT = 5           # delivered, waiting for next patient assignment
    SEEK_EXIT = 6           # all delivered, heading for parking
    PARKING = 7             # inside parking area
    DONE = 8


# =============================================================================
# SERVER LINK - owns the uid/ack handshake
# =============================================================================

class ServerLink:
    """
    Encapsulates the Municipality Server protocol.

    Outgoing: assign a rolling uid, publish, and retry until acked.
    Incoming: auto-acknowledge any non-ack message addressed to us, and
              surface its payload to the mission logic.

    Missing an ack is fatal (server declares Connection Lost after 5 retries),
    so this class is intentionally the only place that touches uid bookkeeping.
    """

    def __init__(self, publisher, logger):
        self._pub = publisher
        self._log = logger
        self._uid = 0
        self._pending = None        # (uid, text, sent_at, retries)

    def _next_uid(self):
        self._uid = (self._uid + 1) % 256
        return self._uid

    def send(self, text):
        """Send a payload and track it until acknowledged."""
        uid = self._next_uid()
        self._publish(uid, text, ack=0)
        self._pending = [uid, text, time.time(), 0]
        self._log.info(f"[SERVER] TX uid={uid} msg='{text}'")

    def send_ack(self, uid):
        """Acknowledge a server message. Must echo the server's uid."""
        self._publish(uid, "", ack=1)
        self._log.info(f"[SERVER] ACK uid={uid}")

    def _publish(self, uid, text, ack):
        m = ServerCommunication()
        m.src = SRC_BUGGY
        m.dest = DEST_SERVER
        m.uid = int(uid) & 0xFF
        m.ack = int(ack)
        m.msg = str(text)
        self._pub.publish(m)

    def handle(self, msg):
        """
        Process an incoming message.
        Returns the payload string if this is a new instruction, else None.
        """
        if msg.dest != ID_BUGGY:
            return None                     # not addressed to us

        if msg.ack == 1:
            if self._pending and msg.uid == self._pending[0]:
                self._log.info(f"[SERVER] our uid={msg.uid} acknowledged")
                self._pending = None
            return None

        # A real instruction from the server - acknowledge immediately.
        self.send_ack(msg.uid)
        payload = (msg.msg or "").strip()
        self._log.info(f"[SERVER] RX uid={msg.uid} msg='{payload}'")
        return payload if payload else None

    def tick(self):
        """Call periodically: retransmit anything left unacknowledged."""
        if not self._pending:
            return
        uid, text, sent_at, retries = self._pending
        if time.time() - sent_at < ACK_TIMEOUT_SEC:
            return
        if retries >= MAX_RETRIES:
            self._log.error(f"[SERVER] uid={uid} unacked after {retries} retries")
            self._pending = None
            return
        self._publish(uid, text, ack=0)
        self._pending = [uid, text, time.time(), retries + 1]
        self._log.warn(f"[SERVER] retry {retries + 1} uid={uid}")


# =============================================================================
# MAIN NODE
# =============================================================================

class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        # ---------------- subscriptions ----------------
        self.create_subscription(EdgeVectors, '/edge_vectors',
                                 self.edge_vectors_callback, QOS)
        self.create_subscription(LaserScan, '/scan',
                                 self.lidar_callback, QOS)
        self.create_subscription(ServerCommunication, '/ServerCommunication',
                                 self.server_communication_callback, QOS)
        self.create_subscription(String, '/qr_detection',
                                 self.qr_detection_callback, QOS)
        self.create_subscription(String, '/sign_board_detection',
                                 self.sign_board_callback, QOS)

        # ---------------- publishers ----------------
        self.publisher_joy = self.create_publisher(Joy, '/cerebri/in/joy', QOS)
        self.publisher_server = self.create_publisher(
            ServerCommunication, '/ServerCommunication', QOS)

        self.server = ServerLink(self.publisher_server, self.get_logger())

        # ---------------- control outputs ----------------
        self.target_speed = 0.0
        self.target_turn = 0.0

        # ---------------- perception state ----------------
        self.lane_turn = 0.0            # steering suggested by lane following
        self.lane_visible = False
        self.front_dist = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        self.obstacle_block = False

        self.last_qr = None
        self.last_qr_time = 0.0
        self.last_sign = None
        self.last_sign_time = 0.0

        # ---------------- mission state ----------------
        self.state = State.INIT
        self.target_building = None     # e.g. "PATIENT_1"
        self.assigned_hospital = None
        self.delivered = 0
        self.state_entered = time.time()
        self.parked_sent = False

        # 20 Hz control loop
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Runner up. Mission state machine armed.")

    # =========================================================================
    # STATE HELPERS
    # =========================================================================

    def set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(
                f"[STATE] {self.state.name} -> {new_state.name}")
            self.state = new_state
            self.state_entered = time.time()

    def time_in_state(self):
        return time.time() - self.state_entered

    def qr_is_fresh(self):
        return (self.last_qr is not None
                and (time.time() - self.last_qr_time) < QR_FRESH_SEC)

    def at_building_wall(self):
        """Conservative proxy for 'inside the building zone'."""
        return self.front_dist < ZONE_WALL_DIST

    def in_zone_for(self, building):
        """Only transmit when BOTH proximity and a fresh matching QR agree."""
        return (self.at_building_wall()
                and self.qr_is_fresh()
                and self.last_qr == building)

    # =========================================================================
    # PERCEPTION CALLBACKS
    # =========================================================================

    def edge_vectors_callback(self, message):
        """
        Convert lane boundary vectors into a steering suggestion.

        vector_count == 2 : both boundaries visible -> steer toward midpoint
        vector_count == 1 : one boundary -> push away from it
        vector_count == 0 : lane lost -> gentle search arc
        """
        width = float(message.image_width) if message.image_width else 640.0
        half = width / 2.0

        if message.vector_count == 0:
            self.lane_visible = False
            self.lane_turn = LANE_LOST_TURN * (1.0 if self.lane_turn >= 0 else -1.0)
            return

        self.lane_visible = True

        if message.vector_count == 2:
            # Midpoint between the inner edges of the two boundaries.
            mid = (message.vector_1[1].x + message.vector_2[0].x) / 2.0
            deviation = half - mid
            self.lane_turn = LANE_KP * (deviation / half)

        else:
            # Single boundary: steer away from whichever side it sits on.
            vx = (message.vector_1[0].x + message.vector_1[1].x) / 2.0
            if vx < half:
                self.lane_turn = -SINGLE_LINE_PUSH   # line on left -> go right
            else:
                self.lane_turn = SINGLE_LINE_PUSH    # line on right -> go left

        self.lane_turn = max(min(self.lane_turn, TURN_MAX), -TURN_MAX)

    def lidar_callback(self, message):
        """Extract front clearance and left/right escape room from the scan."""
        ranges = list(message.ranges)
        n = len(ranges)
        if n == 0:
            return

        def sector_min(center_deg, half_width_deg):
            """Minimum valid range in an angular sector centred on the nose."""
            per_deg = n / 360.0
            lo = int((center_deg - half_width_deg) * per_deg) % n
            hi = int((center_deg + half_width_deg) * per_deg) % n
            if lo <= hi:
                window = ranges[lo:hi + 1]
            else:
                window = ranges[lo:] + ranges[:hi + 1]
            valid = [r for r in window
                     if r is not None and not math.isinf(r)
                     and not math.isnan(r) and r > 0.05]
            return min(valid) if valid else float('inf')

        # Index 0 is assumed to face forward; adjust if the scan is offset.
        self.front_dist = sector_min(0, FRONT_ARC_DEG)
        self.left_clear = sector_min(90, SIDE_ARC_DEG)
        self.right_clear = sector_min(270, SIDE_ARC_DEG)
        self.obstacle_block = self.front_dist < OBSTACLE_STOP_DIST

    def qr_detection_callback(self, message):
        """Normalise a QR payload such as '{LOC: PATIENT_1}' to 'PATIENT_1'."""
        raw = (message.data or "").strip()
        match = BUILDING_RE.search(raw.upper())
        if not match:
            return
        building = match.group(1).upper()
        if building != self.last_qr:
            self.get_logger().info(f"[QR] {building}  (raw='{raw}')")
        self.last_qr = building
        self.last_qr_time = time.time()

    def sign_board_callback(self, message):
        """
        Record the latest sign board reading.

        Expected payload once the classifier is implemented, e.g. "A:LEFT".
        Until then this node publishes nothing and routing falls back to
        exploration - which is by design, not an oversight.
        """
        data = (message.data or "").strip().upper()
        if not data:
            return
        self.last_sign = data
        self.last_sign_time = time.time()
        self.get_logger().info(f"[SIGN] {data}")

    def server_communication_callback(self, message):
        """Route server payloads into the mission state machine."""
        payload = self.server.handle(message)
        if payload is None:
            return

        upper = payload.upper()

        # Server may reply with a letter code or a full building name.
        if upper in CODE_TO_NAME:
            upper = CODE_TO_NAME[upper]

        if upper == "INVALID":
            self.get_logger().error(
                "[SERVER] INVALID - transmitted outside a valid zone")
            return

        if upper == "OK":
            self.get_logger().info("[SERVER] parking confirmed")
            self.set_state(State.DONE)
            return

        if upper.startswith("HOSPITAL"):
            self.assigned_hospital = upper
            self.target_building = upper
            self.get_logger().info(f"[MISSION] assigned -> {upper}")
            self.set_state(State.SEEK_HOSPITAL)
            return

        if upper.startswith("PATIENT"):
            self.target_building = upper
            self.get_logger().info(f"[MISSION] next patient -> {upper}")
            self.set_state(State.SEEK_PATIENT)
            return

    # =========================================================================
    # CONTROL LOOP
    # =========================================================================

    def control_loop(self):
        self.server.tick()

        if self.state == State.INIT:
            # No target yet: explore and scan whatever we encounter.
            self.target_building = None
            self.set_state(State.SEEK_PATIENT)

        elif self.state in (State.SEEK_PATIENT, State.SEEK_HOSPITAL):
            self.drive_seeking()

        elif self.state == State.AT_BUILDING:
            self.drive_stop()
            self.handle_at_building()

        elif self.state in (State.WAIT_ASSIGNMENT, State.WAIT_NEXT):
            # Hold position inside the zone until the server replies.
            self.drive_stop()

        elif self.state == State.SEEK_EXIT:
            self.drive_seeking()
            if self.time_in_state() > 5.0 and self.at_building_wall() is False:
                self.set_state(State.PARKING)

        elif self.state == State.PARKING:
            self.drive_stop()
            if not self.parked_sent:
                self.server.send("PARKED")
                self.parked_sent = True

        elif self.state == State.DONE:
            self.drive_stop()

        self.publish_drive_commands()

    def drive_seeking(self):
        """Lane-follow with reactive obstacle avoidance layered on top."""
        turn = self.lane_turn

        if self.obstacle_block:
            # Blocked: stop forward motion and rotate toward open space.
            turn = 0.6 if self.left_clear > self.right_clear else -0.6
            self.set_control(SPEED_STOP, turn)
            return

        if self.front_dist < OBSTACLE_SLOW_DIST:
            # Something ahead: bias the steer toward the roomier side.
            bias = 0.4 if self.left_clear > self.right_clear else -0.4
            turn = max(min(turn + bias, TURN_MAX), -TURN_MAX)

        # If a fresh QR shows our target, slow down for the approach.
        if self.qr_is_fresh() and self.last_qr == self.target_building:
            self.set_control(SPEED_APPROACH, turn)
            if self.in_zone_for(self.target_building):
                self.set_state(State.AT_BUILDING)
            return

        # Speed schedule: bleed speed proportionally to steering effort.
        speed = SPEED_CRUISE - TURN_SLOWDOWN_GAIN * abs(turn) * (
            SPEED_CRUISE - SPEED_CORNER)
        self.set_control(speed, turn)

    def handle_at_building(self):
        """We are stopped inside a zone with a confirmed QR. Transmit."""
        if not self.in_zone_for(self.target_building):
            # Lost confidence - back out rather than risk an INVALID penalty.
            self.set_state(
                State.SEEK_HOSPITAL if self.assigned_hospital
                else State.SEEK_PATIENT)
            return

        building = self.last_qr

        if building.startswith("PATIENT"):
            self.server.send(building)
            self.set_state(State.WAIT_ASSIGNMENT)

        elif building.startswith("FAKE"):
            self.get_logger().warn("[MISSION] fake hospital - not transmitting")
            self.set_state(State.SEEK_HOSPITAL)

        elif building.startswith("HOSPITAL"):
            if building != self.assigned_hospital:
                self.get_logger().warn(
                    f"[MISSION] {building} is not the assigned "
                    f"{self.assigned_hospital} - skipping")
                self.set_state(State.SEEK_HOSPITAL)
                return
            self.server.send(building)
            self.delivered += 1
            self.get_logger().info(
                f"[MISSION] delivered {self.delivered}/{TOTAL_PATIENTS}")
            self.assigned_hospital = None
            if self.delivered >= TOTAL_PATIENTS:
                self.set_state(State.SEEK_EXIT)
            else:
                self.set_state(State.WAIT_NEXT)

    # =========================================================================
    # ACTUATION
    # =========================================================================

    def set_control(self, speed, turn):
        self.target_speed = float(max(min(speed, SPEED_MAX), -SPEED_MAX))
        self.target_turn = float(max(min(turn, TURN_MAX), -TURN_MAX))

    def drive_stop(self):
        self.set_control(SPEED_STOP, 0.0)

    def rover_move_manual_mode(self, speed, turn):
        """Kept for API compatibility with the shipped skeleton."""
        self.set_control(speed, turn)

    def publish_drive_commands(self):
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg.axes = [0.0, self.target_speed, 0.0, self.target_turn]
        self.publisher_joy.publish(msg)


# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
