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
from nav_msgs.msg import Odometry
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
SPEED_CRUISE = 0.75          # straight-line cruise (time is percentile-scored)
SPEED_CORNER = 0.35          # when steering hard
SPEED_APPROACH = 0.30        # closing on a building to scan
SPEED_STOP = 0.0
TURN_SLOWDOWN_GAIN = 0.6     # how much |turn| bleeds speed

# --- lane following (PD + smoothing) ---
LANE_KP = 0.85               # proportional gain on normalised deviation
LANE_KD = 0.30               # derivative gain - damps the oscillation
TURN_SMOOTHING = 0.55        # EMA weight on previous turn (0 = none, 0.9 = heavy)
LANE_LOST_DECAY = 0.92       # momentum memory: decay last turn while blind
LANE_LOST_MAX_SEC = 1.5      # after this long with no lane, stop guessing
YAW_HOLD_KP = 0.9            # P gain holding the last known heading when blind

# Lane width is LEARNED whenever both boundaries are visible, then reused when
# only one is visible. This keeps the single-boundary case continuous with the
# two-boundary case instead of stepping to a fixed push value.
LANE_WIDTH_INIT_FRAC = 0.70  # initial guess as a fraction of image width
LANE_WIDTH_LEARN_RATE = 0.10 # EMA rate for the learned width

# --- obstacle avoidance ---
OBSTACLE_STOP_DIST = 0.55    # metres, emergency hard-avoid
OBSTACLE_SLOW_DIST = 1.10    # metres, begin proportional evasive steer
OBSTACLE_BIAS_MAX = 0.55     # max steering added by avoidance (was fixed 0.4)
FRONT_ARC_DEG = 22           # +/- degrees treated as "front path" (narrow!)
SIDE_ARC_DEG = 55            # sector used to pick a dodge direction
ZONE_ARC_DEG = 85            # wide arc used ONLY for "am I beside a building"
DEBUG_LIDAR = True           # print front/side clearances at 2 Hz for tuning

# --- zone estimation (penalty-critical: keep conservative) ---
# --- zone estimation ---------------------------------------------------------
# Measured: pyzbar decodes the sign boards reliably out to ~2.35 m (three
# trials: 2.36 / 2.34 / 2.33). We deliberately gate WELL INSIDE that, at 1.7 m,
# rather than transmitting the instant a code becomes readable. Two reasons:
# transmitting outside a zone is a scored penalty and the zone boundary is
# invisible, so margin is cheap insurance; and the approach speed has been
# raised to compensate, so the extra distance costs little time.
ZONE_WALL_DIST = 1.70
QR_FRESH_SEC = 1.5           # a QR read older than this is not trusted

# --- arrival / approach --------------------------------------------------------
# Reading a building's QR is itself strong evidence of being in its zone: the
# code only decodes within ~2.35 m, the board faces the road, and the zone
# images show the zone covering the road frontage. So once we have been reading
# the TARGET's code continuously while crawling, we transmit even if the
# distance gate never trips. Sailing past scores nothing at all, which is far
# worse than transmitting from slightly off-centre.
APPROACH_COMMIT_SEC = 3.0    # sustained approach that counts as arrival
APPROACH_MAX_SEC = 8.0       # hard cap on a single approach
APPROACH_STALL_SEC = 1.5     # secondary trigger: closing has stopped improving
APPROACH_PROGRESS_M = 0.05   # closing less than this does not count as progress
APPROACH_STEER_MAX = 0.45    # authority to steer toward the building while closing
APPROACH_ARC_DEG = 75        # arc searched for the building we are pulling up to
# Once we commit to a building we do NOT abandon it the moment the code leaves
# frame. At cruise the buggy gets only a short burst of decodes as it comes
# level with a board; if losing freshness cancelled the approach it would
# accelerate away from a building it had already decided to visit. Instead we
# hold the commitment and crawl, which nearly always lets the decode return.
APPROACH_QR_GRACE_SEC = 4.0  # keep approaching this long after the code drops out
APPROACH_REACQUIRE_SPEED = 0.14   # crawl while waiting for the code to return

# --- corner safety -------------------------------------------------------------
# Running wide in a bend puts a wheel over the black boundary, which is a scored
# penalty every time. Deviation is a better predictor of that than steering
# effort alone, so large deviation forces a hard slowdown regardless of what the
# smoothed steering command currently reads.
DEVIATION_SLOW = 0.35        # |normalised deviation| that triggers hard slowdown
SPEED_TURN_HARD = 0.26       # speed used when deviation says we are running wide
LANE_KP_FAR = 1.35           # proportional gain once deviation exceeds DEVIATION_SLOW

# --- server protocol ---
ACK_TIMEOUT_SEC = 1.0        # wait before resending an unacked message
MAX_RETRIES = 4              # our own retry budget (server allows 5)
SRC_BUGGY = 1
DEST_SERVER = 2
ID_BUGGY = 1

# --- stuck detection and recovery ---------------------------------------------
# The buggy has no reverse in its normal control path: the avoidance layer
# steers away from obstacles but still commands forward speed. Nosed into a
# corner that is a trap - it grinds against the wall indefinitely. These
# constants drive an explicit escape manoeuvre.
STUCK_SPEED_MIN = 0.10       # we believe we are driving if commanded above this
STUCK_MOVE_MIN = 0.25        # metres of travel expected within the window
STUCK_WINDOW_SEC = 3.0       # no progress for this long => stuck
REVERSE_SPEED = -0.35        # backing-out speed
REVERSE_SEC = 2.0            # how long to reverse
REVERSE_TURN = 0.55          # steer while reversing, to change approach angle
RECOVERY_COOLDOWN_SEC = 4.0  # ignore stuck detection just after a recovery

# --- sign-based routing --------------------------------------------------------
# The detector publishes a table like "A:LEFT,B:RIGHT,C:STRAIGHT". We keep the
# most recent table, look up whichever building we are seeking, and LATCH the
# indicated turn. The latch is spent at the next intersection - detected as the
# lane vanishing, which is precisely when lane following has no information and
# previously just coasted.
SIGN_TABLE_TTL_SEC = 30.0    # a sighting stays useful while we drive to the junction
TURN_COMMIT_STEER = 0.75     # steering magnitude for a committed turn
TURN_COOLDOWN_SEC = 8.0      # after turning, ignore the same instruction briefly

# Waiting for the lane to vanish before acting on a sign is too late at many
# junctions: a T-junction or fork often keeps one boundary in view the whole
# way through, so the commit never fires and we follow whichever edge happens
# to be visible - possibly down the wrong branch. LiDAR gives a much earlier
# signal: on a normal street both sides are bounded by buildings, but where a
# road branches, that side's clearance jumps. We use that to start easing over
# BEFORE the geometry runs out.
JUNCTION_OPENING_DIST = 4.0  # side clearance that indicates a branching road
TURN_EARLY_BIAS = 0.40       # steering applied while easing into the turn
TURN_DONE_YAW_DEG = 55.0     # rotation that counts as "turn completed"
LANE_EFFORT_FULL = 0.55      # |lane_turn| at which lane keeping fully overrides

# --- navigation --------------------------------------------------------------
# Greedy bearing-following, NOT path planning. Buildings are logged with the
# odometry pose at which their QR was read, so we can bias steering toward a
# known target's bearing.
#
# Hard-won rule: while the lane is visible, FOLLOW THE LANE. The lane is the
# road, and leaving it is a scored penalty. An earlier version applied a small
# bearing bias even with the lane in view, and it steadily dragged the buggy
# off the road toward a remembered building - straight through whatever lay in
# between. Bearing now only breaks ties where the road genuinely forks and the
# geometry says nothing, and even there a sign instruction outranks it.
GOAL_BIAS_MAX = 0.45         # authority at a junction with no sign to follow
GOAL_BIAS_LANE = 0.0         # authority while the lane is visible: none
GOAL_REACHED_DIST = 3.0      # metres; close enough to hand over to the QR gate
BEARING_DEADZONE_DEG = 20    # ignore small bearing errors, avoids twitching

# --- mission ---
WAIT_REPLY_TIMEOUT_SEC = 20.0   # give up waiting for the server and resume
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
    RECOVERY = 9            # wedged: reverse out, then resume previous state


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
        self.create_subscription(Odometry, '/cerebri/out/odometry',
                                 self.odometry_callback, QOS)

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
        self.lane_confidence = 0        # 0 = blind, 1 = one edge, 2 = both
        self.lane_width_px = None       # learned online from two-edge frames
        self.prev_deviation = 0.0
        self.prev_lane_time = None
        self.lane_lost_since = None
        self.lane_lost_yaw = None
        self.front_dist = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        self.nearest_dist = float('inf')
        self.nearest_bearing = 0.0
        self.lane_deviation = 0.0
        self.approach_since = None
        self.approach_target = None
        self.approach_best = float('inf')
        self.approach_best_time = 0.0
        self.registered = set()      # patients already transmitted to the server
        self.obstacle_block = False
        self._last_lidar_log = 0.0
        self._last_zone_log = 0.0

        self.last_qr = None
        self.last_qr_time = 0.0
        self.last_sign = None
        self.last_sign_time = 0.0

        # ---------------- sign routing ----------------
        self.routing_table = {}       # letter code -> 'LEFT' | 'RIGHT' | 'STRAIGHT'
        self.routing_time = 0.0
        self.pending_turn = None      # latched instruction for the next junction
        self.pending_turn_yaw = None  # heading when it was latched
        self.turn_committed_at = 0.0
        self.was_lane_visible = True

        # ---------------- navigation state ----------------
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.have_pose = False
        # building name -> (x, y) where its QR was successfully read
        self.building_map = {}

        # ---------------- stuck detection ----------------
        self.last_progress_x = 0.0
        self.last_progress_y = 0.0
        self.last_progress_time = time.time()
        self.recovery_until = 0.0
        self.recovery_return_state = None
        self.last_recovery_time = 0.0
        self.recovery_turn_sign = 1.0

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

    def effective_target(self):
        """
        The building we are currently trying to reach.

        At mission start (and after each delivery) we have no assignment yet,
        so ANY patient building is a legitimate target - that is how the first
        patient is found. Once the server assigns a hospital, that becomes the
        target until the delivery completes.
        """
        if self.target_building is not None:
            return self.target_building
        # Unassigned: any patient we have NOT already registered is fair game.
        # Without the registered check the buggy re-stops at a patient it has
        # already transmitted, waits out the reply timeout, and loses ~20 s.
        if (self.last_qr and self.last_qr.startswith("PATIENT")
                and self.last_qr not in self.registered):
            return self.last_qr
        return None

    def odometry_callback(self, message):
        """Track pose so building sightings can be geo-referenced."""
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        self.pose_x = p.x
        self.pose_y = p.y
        # Yaw from quaternion (planar robot, so only z/w matter materially).
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose_yaw = math.atan2(siny, cosy)
        self.have_pose = True

    def remember_building(self, name):
        """Record where a building was seen, the first time we see it."""
        if not self.have_pose or name in self.building_map:
            return
        self.building_map[name] = (self.pose_x, self.pose_y)
        self.get_logger().info(
            f"[MAP] {name} logged at ({self.pose_x:.1f}, {self.pose_y:.1f}) "
            f"- {len(self.building_map)} building(s) known")

    def goal_bearing_error(self, name):
        """
        Signed heading error to a known building, in radians.
        Positive means the goal lies to our left. None if unknown/arrived.
        """
        if not self.have_pose or name not in self.building_map:
            return None
        gx, gy = self.building_map[name]
        dx, dy = gx - self.pose_x, gy - self.pose_y
        if math.hypot(dx, dy) < GOAL_REACHED_DIST:
            return None                      # close enough; let QR take over
        desired = math.atan2(dy, dx)
        err = desired - self.pose_yaw
        # normalise to [-pi, pi]
        return math.atan2(math.sin(err), math.cos(err))

    def goal_steer_bias(self, name):
        """Steering contribution that turns us toward a known target."""
        # A latched sign instruction is authoritative - it came from the road
        # network itself, whereas bearing is a straight line that knows nothing
        # about walls. Never let bearing argue with a sign.
        if self.pending_turn is not None:
            return 0.0
        # With the lane in view we follow the lane, full stop.
        if self.lane_confidence >= 1:
            return 0.0

        err = self.goal_bearing_error(name)
        if err is None:
            return 0.0
        if abs(err) < math.radians(BEARING_DEADZONE_DEG):
            return 0.0
        return max(min(err / math.pi * 2.0, 1.0), -1.0) * GOAL_BIAS_MAX

    def is_stuck(self):
        """
        True when we are commanding forward motion but not actually moving.

        Compares travelled distance against a time window rather than reading
        velocity, because a wheel grinding against a wall can still report
        motion. Only meaningful when we intend to be driving.
        """
        if not self.have_pose:
            return False
        if self.target_speed < STUCK_SPEED_MIN:
            # Not trying to move (e.g. waiting in a zone) - not stuck.
            self.last_progress_x = self.pose_x
            self.last_progress_y = self.pose_y
            self.last_progress_time = time.time()
            return False
        if time.time() - self.last_recovery_time < RECOVERY_COOLDOWN_SEC:
            return False

        moved = math.hypot(self.pose_x - self.last_progress_x,
                           self.pose_y - self.last_progress_y)
        if moved > STUCK_MOVE_MIN:
            self.last_progress_x = self.pose_x
            self.last_progress_y = self.pose_y
            self.last_progress_time = time.time()
            return False

        return (time.time() - self.last_progress_time) > STUCK_WINDOW_SEC

    def enter_recovery(self):
        """Begin a timed reverse manoeuvre, remembering where to return to."""
        if self.state == State.RECOVERY:
            return
        self.recovery_return_state = self.state
        self.recovery_until = time.time() + REVERSE_SEC
        # Reverse away from whichever side has less room, so we rotate toward
        # the opening rather than back into the same trap.
        self.recovery_turn_sign = -1.0 if self.left_clear < self.right_clear else 1.0
        self.get_logger().warn(
            f"[RECOVERY] stuck detected - reversing out "
            f"(front={self.front_dist:.2f} L={self.left_clear:.2f} "
            f"R={self.right_clear:.2f})")
        self.set_state(State.RECOVERY)

    def drive_recovery(self):
        """Back up with steering, then hand control back."""
        if time.time() >= self.recovery_until:
            self.last_recovery_time = time.time()
            self.last_progress_x = self.pose_x
            self.last_progress_y = self.pose_y
            self.last_progress_time = time.time()
            back_to = self.recovery_return_state or State.SEEK_PATIENT
            self.get_logger().info(f"[RECOVERY] complete -> {back_to.name}")
            self.set_state(back_to)
            return
        self.set_control(REVERSE_SPEED, REVERSE_TURN * self.recovery_turn_sign)

    def qr_is_fresh(self):
        return (self.last_qr is not None
                and (time.time() - self.last_qr_time) < QR_FRESH_SEC)

    def at_building_wall(self):
        """
        Conservative proxy for 'inside the building zone'.

        Uses the WIDE arc, not the forward path: the sign board and building
        face are usually off to one side as we pull alongside, so a narrow
        forward check would report clear road and we would sail past.
        """
        return self.nearest_dist < ZONE_WALL_DIST

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
        Convert lane boundary vectors into a steering command.

        The controller estimates the lane CENTRE in image space, then applies
        PD control on the normalised deviation from the image centre.

        Both-edges case : centre = midpoint of the two boundaries, and the
                          observed lane width is learned.
        Single-edge case: centre = boundary offset by half the LEARNED lane
                          width. This is continuous with the two-edge case, so
                          a boundary leaving frame mid-corner no longer causes
                          a step change in steering.
        No-edge case    : momentum memory - decay the last command rather than
                          snapping to zero, then give up after a timeout.
        """
        width = float(message.image_width) if message.image_width else 320.0
        half = width / 2.0
        now = time.time()

        if self.lane_width_px is None:
            self.lane_width_px = LANE_WIDTH_INIT_FRAC * width

        count = message.vector_count
        self.lane_confidence = count
        centre = None

        if count >= 2:
            x1 = (message.vector_1[0].x + message.vector_1[1].x) / 2.0
            x2 = (message.vector_2[0].x + message.vector_2[1].x) / 2.0
            centre = (x1 + x2) / 2.0

            # Learn the lane width, ignoring implausible observations.
            observed = abs(x2 - x1)
            if 0.25 * width < observed < 1.5 * width:
                r = LANE_WIDTH_LEARN_RATE
                self.lane_width_px = (1.0 - r) * self.lane_width_px + r * observed

        elif count == 1:
            vx = (message.vector_1[0].x + message.vector_1[1].x) / 2.0
            offset = self.lane_width_px / 2.0
            # Boundary left of frame centre -> lane lies to its right.
            centre = vx + offset if vx < half else vx - offset

        if centre is None:
            # Lane lost. This is usually one of two things:
            #   a) a momentary dropout mid-curve  -> keep turning (momentum)
            #   b) an intersection, where there genuinely is no single lane
            #
            # For (a) momentum is right. For (b) the old behaviour - decay the
            # steering to zero - meant "straight relative to the buggy", which
            # drifts, and drifting inside a junction is how excursions start.
            # Instead we latch the heading we had when the lane was last seen
            # and actively hold it in the WORLD frame, so we cross the junction
            # on a stable bearing. Any goal bias is still layered on top in
            # drive_seeking, so a known target can still steer us onto a branch.
            self.lane_visible = False
            if self.lane_lost_since is None:
                self.lane_lost_since = now
                self.lane_lost_yaw = self.pose_yaw if self.have_pose else None

            # A latched sign instruction is spent HERE. The lane vanishing is
            # our intersection detector: it is exactly the moment the geometry
            # stops telling us where to go and the sign has to.
            if self.pending_turn in ('LEFT', 'RIGHT'):
                sign = 1.0 if self.pending_turn == 'LEFT' else -1.0
                self.lane_turn = sign * TURN_COMMIT_STEER
                return

            if now - self.lane_lost_since > LANE_LOST_MAX_SEC:
                if self.have_pose and self.lane_lost_yaw is not None:
                    err = self.lane_lost_yaw - self.pose_yaw
                    err = math.atan2(math.sin(err), math.cos(err))
                    self.lane_turn = max(min(YAW_HOLD_KP * err,
                                             TURN_MAX), -TURN_MAX)
                else:
                    self.lane_turn = 0.0
            else:
                self.lane_turn *= LANE_LOST_DECAY
            self.was_lane_visible = False
            return

        self.lane_visible = True
        self.lane_lost_since = None
        self.lane_lost_yaw = None

        # Lane reacquired. If we were mid-commitment, the junction is behind us
        # and the instruction has been spent - clear it so we do not turn again
        # at the following junction.
        if not self.was_lane_visible and self.pending_turn is not None:
            self.clear_pending_turn("lane reacquired")
        self.was_lane_visible = True

        # --- PD on normalised deviation ---
        deviation = (half - centre) / half          # +ve => lane centre is left
        dt = (now - self.prev_lane_time) if self.prev_lane_time else 0.0
        derivative = ((deviation - self.prev_deviation) / dt) if dt > 1e-3 else 0.0
        self.prev_deviation = deviation
        self.prev_lane_time = now

        # Progressive gain: a small deviation gets a gentle correction, but once
        # we are genuinely running wide the gain rises sharply. Constant gain
        # tuned soft enough to avoid weaving on straights is, by construction,
        # too soft to pull us back from the edge of a bend - which is where the
        # black boundary is.
        kp = LANE_KP if abs(deviation) < DEVIATION_SLOW else LANE_KP_FAR
        raw_turn = kp * deviation + LANE_KD * derivative
        self.lane_deviation = deviation

        # --- EMA smoothing: suppresses the straight-line weave ---
        a = TURN_SMOOTHING
        self.lane_turn = a * self.lane_turn + (1.0 - a) * raw_turn
        self.lane_turn = max(min(self.lane_turn, TURN_MAX), -TURN_MAX)

    def lidar_callback(self, message):
        """
        Extract front clearance and left/right escape room from the scan.

        IMPORTANT: this scan spans angle_min..angle_max (here -pi..+pi), so
        index 0 is NOT straight ahead - forward is wherever angle == 0 falls.
        We therefore map each desired bearing (0 = forward, +90 = left,
        -90 = right, in the sensor frame) to an index via angle_min and
        angle_increment, rather than assuming index 0 faces forward.
        """
        ranges = message.ranges
        n = len(ranges)
        if n == 0:
            return

        angle_min = message.angle_min
        angle_inc = message.angle_increment if message.angle_increment else (
            2.0 * math.pi / n)

        def idx_for(bearing_rad):
            i = int(round((bearing_rad - angle_min) / angle_inc))
            return max(0, min(n - 1, i))

        def sector_min(center_deg, half_width_deg):
            """Minimum valid range in an angular sector about a bearing."""
            c = math.radians(center_deg)
            lo = idx_for(c - math.radians(half_width_deg))
            hi = idx_for(c + math.radians(half_width_deg))
            if lo > hi:
                lo, hi = hi, lo
            window = ranges[lo:hi + 1]
            valid = [r for r in window
                     if r is not None and not math.isinf(r)
                     and not math.isnan(r) and r > 0.05]
            return min(valid) if valid else float('inf')

        def sector_min_bearing(center_deg, half_width_deg):
            """Nearest range in a sector AND the bearing (deg) it was found at."""
            c = math.radians(center_deg)
            lo = idx_for(c - math.radians(half_width_deg))
            hi = idx_for(c + math.radians(half_width_deg))
            if lo > hi:
                lo, hi = hi, lo
            best_r, best_i = float('inf'), None
            for i in range(lo, hi + 1):
                r = ranges[i]
                if (r is None or math.isinf(r) or math.isnan(r) or r <= 0.05):
                    continue
                if r < best_r:
                    best_r, best_i = r, i
            if best_i is None:
                return float('inf'), 0.0
            bearing = math.degrees(angle_min + best_i * angle_inc)
            return best_r, bearing

        # Bearings in the sensor frame: 0 rad = forward, +90 = left, -90 = right.
        self.front_dist = sector_min(0, FRONT_ARC_DEG)
        self.left_clear = sector_min(90, SIDE_ARC_DEG)
        self.right_clear = sector_min(-90, SIDE_ARC_DEG)
        self.obstacle_block = self.front_dist < OBSTACLE_STOP_DIST

        # Buildings sit BESIDE the road, not across it, so "have I arrived at
        # this building" cannot use the narrow front path - driving past a
        # building with it on our left leaves front_dist at infinity. This wide
        # arc answers a different question: is there anything solid close by in
        # roughly the direction we are facing? It is used only for the zone
        # gate and the approach steer, never for obstacle avoidance.
        self.nearest_dist, self.nearest_bearing = sector_min_bearing(
            0, ZONE_ARC_DEG)

        # Throttled debug: prove what the front sector sees. Remove once tuned.
        if DEBUG_LIDAR:
            now = time.time()
            if now - self._last_lidar_log > 0.5:
                self._last_lidar_log = now
                self.get_logger().info(
                    f"[LIDAR] front={self.front_dist:.2f} "
                    f"near={self.nearest_dist:.2f} "
                    f"L={self.left_clear:.2f} R={self.right_clear:.2f} "
                    f"block={self.obstacle_block}")

    def qr_detection_callback(self, message):
        """Normalise a QR payload such as '{LOC: PATIENT_1}' to 'PATIENT_1'."""
        raw = (message.data or "").strip()
        match = BUILDING_RE.search(raw.upper())
        if not match:
            return
        building = match.group(1).upper()
        if building != self.last_qr:
            self.get_logger().info(f"[QR] {building}  (raw='{raw}')")
        self.remember_building(building)
        self.last_qr = building
        self.last_qr_time = time.time()

    def sign_board_callback(self, message):
        """
        Consume a routing table such as "A:LEFT,B:RIGHT,C:STRAIGHT".

        One board carries several destinations, so we store the whole table and
        look up whichever building we happen to be seeking. The instruction is
        LATCHED rather than acted on immediately: the sign becomes readable well
        before the junction, so it must be remembered until the lane actually
        disappears.
        """
        data = (message.data or "").strip().upper()
        if not data:
            return

        table = {}
        for part in data.split(','):
            if ':' not in part:
                continue
            code, direction = part.split(':', 1)
            code, direction = code.strip(), direction.strip()
            if code in NAME_TO_CODE.values() and direction in (
                    'LEFT', 'RIGHT', 'STRAIGHT'):
                table[code] = direction

        if not table:
            return

        self.routing_table = table
        self.routing_time = time.time()
        if data != self.last_sign:
            self.get_logger().info(f"[SIGN] {data}")
        self.last_sign = data
        self.last_sign_time = time.time()

        self.latch_turn_for_target()

    def latch_turn_for_target(self):
        """If the current target appears in the routing table, remember its turn."""
        target = self.effective_target()
        if not target:
            return
        code = NAME_TO_CODE.get(target)
        if not code:
            return
        if time.time() - self.routing_time > SIGN_TABLE_TTL_SEC:
            return
        if time.time() - self.turn_committed_at < TURN_COOLDOWN_SEC:
            return

        direction = self.routing_table.get(code)
        if direction and direction != self.pending_turn:
            self.pending_turn = direction
            self.pending_turn_yaw = self.pose_yaw if self.have_pose else None
            self.get_logger().info(
                f"[ROUTE] {target} ({code}) -> {direction} at next junction")

    def junction_turn_bias(self):
        """
        Ease toward a latched turn as soon as a junction is DETECTED, rather
        than waiting for the lane to disappear.

        The junction signal is a side opening: clearance on that side much
        larger than a street lined with buildings would give.

        PRIORITY RULE - this is the important part. Routing is a PREFERENCE;
        staying inside the lane is a CONSTRAINT. So whenever the routing nudge
        opposes what the lane controller is asking for, it is attenuated in
        proportion to how hard the lane controller is working. Drifting toward
        the correct branch on a wide-open junction is free; doing it while the
        lane controller is fighting to keep us off a boundary is how you leave
        the road. At full lane effort the routing bias vanishes entirely.
        """
        if self.pending_turn not in ('LEFT', 'RIGHT'):
            return 0.0

        side = (self.left_clear if self.pending_turn == 'LEFT'
                else self.right_clear)
        if side < JUNCTION_OPENING_DIST:
            return 0.0                      # no branch that way yet

        sign = 1.0 if self.pending_turn == 'LEFT' else -1.0
        scale = 1.0 if self.lane_confidence <= 1 else 0.5

        # Lane keeping outranks routing preference.
        if sign * self.lane_turn < 0:       # bias opposes the lane correction
            effort = min(abs(self.lane_turn) / LANE_EFFORT_FULL, 1.0)
            scale *= (1.0 - effort)

        return sign * TURN_EARLY_BIAS * scale

    def turn_completed(self):
        """True once we have rotated far enough to call the turn done."""
        if self.pending_turn is None or self.pending_turn_yaw is None:
            return False
        if not self.have_pose:
            return False
        err = self.pose_yaw - self.pending_turn_yaw
        err = math.atan2(math.sin(err), math.cos(err))
        return abs(math.degrees(err)) > TURN_DONE_YAW_DEG

    def clear_pending_turn(self, reason):
        if self.pending_turn is None:
            return
        self.get_logger().info(f"[ROUTE] {self.pending_turn} done ({reason})")
        self.pending_turn = None
        self.pending_turn_yaw = None
        self.turn_committed_at = time.time()

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
            # We transmitted from outside a valid zone. Staying parked here
            # waiting for an assignment that will never come would end the run,
            # so back out and approach again.
            self.get_logger().error(
                "[SERVER] INVALID - transmitted outside a valid zone, resuming")
            self.set_state(State.SEEK_HOSPITAL if self.assigned_hospital
                           else State.SEEK_PATIENT)
            return

        if upper == "OK":
            self.get_logger().info("[SERVER] parking confirmed")
            self.set_state(State.DONE)
            return

        if upper.startswith("HOSPITAL"):
            self.assigned_hospital = upper
            self.target_building = upper
            self.get_logger().info(f"[MISSION] assigned -> {upper}")
            self.pending_turn = None
            self.latch_turn_for_target()    # we may already hold a useful sign
            self.set_state(State.SEEK_HOSPITAL)
            return

        if upper.startswith("PATIENT"):
            self.target_building = upper
            self.get_logger().info(f"[MISSION] next patient -> {upper}")
            self.pending_turn = None
            self.latch_turn_for_target()
            self.set_state(State.SEEK_PATIENT)
            return

    # =========================================================================
    # CONTROL LOOP
    # =========================================================================

    def control_loop(self):
        self.server.tick()

        # Stuck detection runs above the state machine: being wedged against a
        # wall is possible from any driving state, and no other behaviour can
        # escape it because the normal control path never commands reverse.
        if self.state not in (State.RECOVERY, State.DONE,
                              State.WAIT_ASSIGNMENT, State.WAIT_NEXT):
            if self.is_stuck():
                self.enter_recovery()

        if self.state == State.RECOVERY:
            self.drive_recovery()
            self.publish_drive_commands()
            return

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
            # Hold position inside the zone until the server replies - leaving
            # early is a scored penalty. But never wait forever: a dropped
            # message would otherwise park the buggy for the rest of the run,
            # which costs far more than the risk of moving on.
            self.drive_stop()
            if self.time_in_state() > WAIT_REPLY_TIMEOUT_SEC:
                self.get_logger().warn(
                    f"[MISSION] no server reply in {WAIT_REPLY_TIMEOUT_SEC:.0f}s "
                    f"- resuming search")
                self.set_state(State.SEEK_HOSPITAL if self.assigned_hospital
                               else State.SEEK_PATIENT)

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
        """
        Lane following with reactive avoidance blended in (not overriding).

        Key idea: only the NARROW forward path should trigger a dodge. A wall
        we are driving alongside is close but irrelevant, so side/near readings
        must not yank the wheel. The avoidance term is proportional to how close
        and how central the obstacle is, and it is ADDED to the lane command so
        the buggy keeps tracking the lane while it eases around the object.
        """
        turn = self.lane_turn

        # Ease onto the branch a sign told us to take, as soon as LiDAR shows
        # the branch exists. This runs while the lane is still partly visible,
        # which is exactly the case the lane-loss commit misses.
        turn = max(min(turn + self.junction_turn_bias(), TURN_MAX), -TURN_MAX)

        # A completed rotation retires the instruction even if the lane never
        # fully vanished, so we do not carry it into the following junction.
        if self.turn_completed():
            self.clear_pending_turn("rotation complete")

        # Navigation: if we know where the target is, bias toward its bearing.
        # Authority is low while the lane is clearly visible and high when it
        # is not - i.e. this mostly decides which way to go at intersections,
        # which is exactly where lane following has no information.
        #
        # Suppressed entirely when something is close ahead: greedy bearing
        # following has no path planning, so without this it will happily drive
        # into a wall that happens to lie between us and the goal.
        target_now = self.effective_target()
        if target_now and self.front_dist > OBSTACLE_SLOW_DIST:
            turn = max(min(turn + self.goal_steer_bias(target_now),
                           TURN_MAX), -TURN_MAX)

        # Emergency: something very close, dead ahead. Steer hard toward the
        # side with more room but KEEP part of the lane term so we don't rotate
        # blindly across a boundary.
        if self.front_dist < OBSTACLE_STOP_DIST:
            escape = 0.7 if self.left_clear > self.right_clear else -0.7
            turn = max(min(0.5 * self.lane_turn + escape, TURN_MAX), -TURN_MAX)
            self.set_control(SPEED_CORNER, turn)   # crawl, don't fully stop
            return

        # Proportional avoidance: ramps from 0 at OBSTACLE_SLOW_DIST to full
        # at OBSTACLE_STOP_DIST. No fixed slap, so driving parallel to a wall
        # that only clips the far edge of the front arc barely perturbs us.
        if self.front_dist < OBSTACLE_SLOW_DIST:
            span = OBSTACLE_SLOW_DIST - OBSTACLE_STOP_DIST
            severity = (OBSTACLE_SLOW_DIST - self.front_dist) / max(span, 1e-3)
            severity = max(0.0, min(1.0, severity))
            bias = OBSTACLE_BIAS_MAX * severity
            bias = bias if self.left_clear > self.right_clear else -bias
            turn = max(min(turn + bias, TURN_MAX), -TURN_MAX)

        # --- arrival at the target building ------------------------------
        # Entering the approach requires a fresh decode of the target, but
        # STAYING in it does not: see APPROACH_QR_GRACE_SEC. Losing the code
        # for a moment is normal as the board leaves frame, and abandoning the
        # approach there was making the buggy accelerate past buildings it had
        # already committed to.
        target = self.effective_target()
        fresh_on_target = (target and self.qr_is_fresh()
                           and self.last_qr == target)

        if fresh_on_target and self.approach_since is None:
            self.approach_since = time.time()
            self.approach_target = target
            self.approach_best = float('inf')
            self.approach_best_time = time.time()
            self.get_logger().info(f"[APPROACH] closing on {target}")

        if self.approach_since is not None:
            committed = self.approach_target
            since_qr = time.time() - self.last_qr_time
            held = time.time() - self.approach_since

            # Give up only if the code has been gone a long time, or the cap
            # is reached - not merely because this frame had no decode.
            if since_qr > APPROACH_QR_GRACE_SEC or held > APPROACH_MAX_SEC:
                self.get_logger().warn(
                    f"[APPROACH] abandoning {committed} "
                    f"(no code for {since_qr:.1f}s, held {held:.1f}s)")
                self.approach_since = None
                self.approach_target = None
            else:
                if self.nearest_dist < self.approach_best - APPROACH_PROGRESS_M:
                    self.approach_best = self.nearest_dist
                    self.approach_best_time = time.time()

                lean = max(min(self.nearest_bearing / 90.0, 1.0), -1.0)
                turn = max(min(turn + lean * APPROACH_STEER_MAX,
                               TURN_MAX), -TURN_MAX)

                # While the code is missing, crawl. Stopping dead risks never
                # improving the viewing angle; crawling usually re-acquires.
                on_target_now = (self.qr_is_fresh()
                                 and self.last_qr == committed)
                self.set_control(
                    SPEED_APPROACH if on_target_now else APPROACH_REACQUIRE_SPEED,
                    turn)

                stalled = time.time() - self.approach_best_time

                # Arrival trigger. Measured behaviour: 'near' keeps improving
                # right up until the code is lost, because pulling alongside
                # slides the board out of the forward camera's view - so the
                # buggy never "stops closing" while it can still see the code.
                # Waiting for a stall therefore means waiting for QR loss.
                # A sustained approach is the reliable signal, and it is the
                # one that worked: 3.5 s transmitted successfully at two
                # different buildings. Stall is kept as a secondary trigger for
                # the case where we really are blocked.
                arrived = (self.in_zone_for(committed)
                           or held > APPROACH_COMMIT_SEC
                           or (stalled > APPROACH_STALL_SEC and held > 1.0))

                if on_target_now and arrived:
                    self.get_logger().info(
                        f"[APPROACH] closest {self.approach_best:.2f} m "
                        f"after {held:.1f}s - arrived")
                    self.set_state(State.AT_BUILDING)
                else:
                    now_t = time.time()
                    if now_t - self._last_zone_log > 1.0:
                        self._last_zone_log = now_t
                        self.get_logger().info(
                            f"[ZONE] {committed} near={self.nearest_dist:.2f} "
                            f"best={self.approach_best:.2f} "
                            f"qr_age={since_qr:.1f}s held={held:.1f}s")
                return

        # Speed schedule: bleed speed with steering effort, and slow further
        # when lane confidence drops - losing an edge usually means a corner.
        speed = SPEED_CRUISE - TURN_SLOWDOWN_GAIN * abs(turn) * (
            SPEED_CRUISE - SPEED_CORNER)

        # Deviation predicts a boundary excursion better than steering effort
        # does: the EMA smoothing means the steering command lags the error, so
        # by the time |turn| is large we are already wide. Braking on deviation
        # itself reacts earlier.
        if abs(self.lane_deviation) > DEVIATION_SLOW:
            speed = min(speed, SPEED_TURN_HARD)
        if self.lane_confidence <= 1:
            speed = min(speed, SPEED_CORNER)
        if not self.lane_visible:
            speed = min(speed, SPEED_APPROACH)

        self.set_control(speed, turn)

    def handle_at_building(self):
        """We are stopped at a building with a confirmed QR. Transmit."""
        target = self.effective_target()

        # We may have arrived via the distance gate OR via sustained QR contact
        # (see the approach logic), so the requirement here is the one that
        # actually matters for correctness: we are still reading THIS
        # building's code right now. Re-testing the distance gate would bounce
        # us straight back out of every QR-persistence arrival.
        if not (self.qr_is_fresh() and self.last_qr == target):
            self.get_logger().warn(
                "[MISSION] lost QR contact on arrival - backing out")
            self.set_state(
                State.SEEK_HOSPITAL if self.assigned_hospital
                else State.SEEK_PATIENT)
            return

        building = self.last_qr

        if building.startswith("PATIENT"):
            # Protocol uses single-letter codes on the wire (A/B/C), while the
            # QR encodes the full name ({LOC: PATIENT_1}). Translate before TX.
            self.server.send(NAME_TO_CODE.get(building, building))
            self.registered.add(building)
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
            self.server.send(NAME_TO_CODE.get(building, building))
            self.delivered += 1
            self.get_logger().info(
                f"[MISSION] delivered {self.delivered}/{TOTAL_PATIENTS}")
            self.assigned_hospital = None
            # Clearing the target matters: leaving it set to the hospital we
            # just delivered to meant that if the next assignment was ever
            # missed, the buggy resumed seeking that same hospital and then
            # refused it forever, because assigned_hospital was already None.
            self.target_building = None
            self.approach_since = None
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