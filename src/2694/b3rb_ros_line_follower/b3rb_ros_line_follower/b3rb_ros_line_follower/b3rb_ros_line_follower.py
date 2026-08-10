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
#   3. The docs contradict each other about server payloads: the official
#      Server_Communication PDF shows single letters on the wire ("A", "X"),
#      while an NXP forum answer says to send the string as read by the QR.
#      Only one is accepted, so we lead with the PDF's letter format and, on an
#      INVALID received while still standing in the zone, retry in place with
#      the next format. The accepted format is latched. See server_payload().
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
SPEED_CRUISE = 0.65          # straight-line cruise (time is percentile-scored).
                             # Restored to the video-era value: the bend
                             # problem was phantom avoidance, not speed, so
                             # there is no reason to pay the time penalty.
SPEED_CORNER = 0.35          # when steering hard
SPEED_APPROACH = 0.30        # closing on a building to scan
SPEED_STOP = 0.0
TURN_SLOWDOWN_GAIN = 0.6     # how much |turn| bleeds speed

# --- lane following (PD + smoothing) ---
LANE_KP = 0.85               # proportional gain on normalised deviation
LANE_KD = 0.30               # derivative gain - damps the oscillation
TURN_SMOOTHING = 0.55        # EMA weight on previous turn (0 = none, 0.9 = heavy)
LANE_LOST_DECAY = 0.97       # momentum memory: decay last turn while blind
LANE_LOST_MAX_SEC = 3.0     # after this long with no lane, stop guessing
YAW_HOLD_KP = 0.9            # P gain holding the last known heading when blind

# Lane width is LEARNED whenever both boundaries are visible, then reused when
# only one is visible. This keeps the single-boundary case continuous with the
# two-boundary case instead of stepping to a fixed push value.
LANE_WIDTH_INIT_FRAC = 0.70  # initial guess as a fraction of image width
LANE_WIDTH_LEARN_RATE = 0.10 # EMA rate for the learned width
# With a single boundary visible we place the lane centre at half a learned
# width from it - but if that learned width is even slightly small, the aim
# point sits too close to the ONE boundary we can actually see, and that is
# precisely the line we are about to touch. Erring outward costs nothing
# (the far side is open, or we would be seeing two edges) and directly buys
# clearance from a known black line.
SINGLE_EDGE_MARGIN = 1.18    # multiplier on the half-width offset

# --- fork handling -------------------------------------------------------
# At a V-fork the detector still reports two boundaries, but they are the
# OUTER edges of two different branches - so their midpoint aims the buggy
# squarely at the divider between them, which is exactly the observed failure
# (drove straight into the V). A fork announces itself as a lane that has
# suddenly become far wider than the learned width; unlike divider islands
# (where width stays normal), a V genuinely produces this signature, so the
# ratio test is reliable HERE even though it was not reliable there.
#
# Response: pick a branch - the sign instruction if one is latched, else the
# wider opening - and aim at the middle of that opening. The choice is LATCHED
# on entry: re-deciding every frame flips which gap is wider as we steer,
# which saws the wheel. Steering applies directly (no EMA), because at a fork
# the smoother is pure lag.
FORK_WIDTH_RATIO = 1.45      # observed/learned width above this = fork
# Measured: 1.30 saturated the command (log showed turn=-1.00 at both forks),
# so the buggy entered the branch at full lock and over-rotated onto the inner
# boundary of the branch it was taking. Lower gain keeps the entry committed
# but no longer pinned, so it tracks into the branch instead of swinging.
FORK_KP = 0.85               # direct gain on the aim point at a fork
FORK_SPEED = 0.22            # crawl: a tight branch entry needs time
FORK_HOLD_SEC = 2.0          # keep fork mode long enough to clear the split

# --- curve inner-line bias -----------------------------------------------
# On a bend, perspective pulls the far end of the INSIDE boundary toward the
# image centre, so the midpoint of the two boundaries is not the lane centre -
# it drifts toward the inside of the curve, and the controller faithfully
# tracks that drifted centre onto the inner black line. Measure the curve from
# the boundary vectors' lean and shift the aim point back toward the outside,
# proportionally. Zero on a straight, so straight-line behaviour is untouched.
# Measured: 0.35 at cruise 0.55 still clipped the inner line, so the
# correction was too weak rather than the speed too high. Raised, with a
# larger cap so the shift is not saturating on tight bends. If the buggy now
# runs wide toward the OUTER line on curves, this is the constant to lower.
# DISABLED by default. This was added to counteract inner-line cutting that
# turned out to be caused by the corridor avoidance dodging phantom obstacles
# on bends (now reverted). With the real cause removed, leaving this at 0.50
# would push the buggy WIDE toward the outer boundary instead. The video-era
# code had no such term and drove cleanly. Raise to ~0.30 only if measurable
# inner-line contact remains after the avoidance revert.
CURVE_LEAN_GAIN = 0.30       # fraction of measured lean applied as outward shift
CURVE_LEAN_MAX_PX = 80.0     # cap on the shift, pixels

# --- obstacle avoidance ---
OBSTACLE_STOP_DIST = 0.55    # metres, emergency hard-avoid
OBSTACLE_SLOW_DIST = 1.10    # metres, begin proportional evasive steer
OBSTACLE_BIAS_MAX = 0.55     # max steering added by avoidance (was fixed 0.4)

# Avoidance must respect the lane. Picking the dodge direction from side-sector
# clearance alone is wrong: those sectors mostly measure the buildings flanking
# the road, so "more room that way" frequently means "off the track that way" -
# which is how dodging a pole put a wheel over the right-hand boundary.
#
# Both outcomes are penalties, so neither can simply win. The lane now gets a
# vote on WHICH WAY to dodge, and when the dodge still ends up opposing the
# lane correction it is attenuated in proportion to how hard the lane
# controller is working - but never below a floor, or we would drive into the
# obstacle instead. The remaining conflict is resolved with the brakes:
# slowing down shrinks the lateral excursion needed to clear the same object.
AVOID_LANE_FLOOR = 0.35      # min fraction of the dodge kept when it fights the lane
AVOID_BRAKE_SPEED = 0.20     # speed while dodging against the lane

# The dodge direction must be LATCHED once chosen. Recomputing it every tick
# is a feedback loop: dodging left makes the lane controller push back right,
# which flips the preferred side, which reverses the dodge, which flips it
# again - the buggy saws left-right-left at 20 Hz, makes no net lateral
# progress, and drives into the obstacle it is trying to clear. Observed
# exactly this. So the side is decided ONCE per encounter and held until the
# path has been clear for a moment.
AVOID_HOLD_SEC = 1.5         # hold the chosen dodge side this long past last trigger

# The lane only gets to pick the dodge side if that side has REAL room, not
# merely more than the trigger distance. Measured failure: obstacle at 1.05 m,
# left clear 7.23 m, right clear 1.52 m - the lane wanted right, 1.52 cleared
# the 1.10 bar, and the buggy squeezed into the tighter gap and clipped. A gap
# barely wider than the trigger range is not somewhere to aim a swerve.
AVOID_SIDE_MIN = 2.20        # side clearance needed before the lane may choose it
AVOID_SIDE_RATIO = 2.0       # ...or if the other side is this many times wider,
                             # take the open side regardless of what the lane wants
# Widened from 22. A thin obstacle - pole, tree trunk - sitting ~25 deg off
# centre fell entirely OUTSIDE a +/-22 deg cone, so avoidance never fired and
# the buggy drove into it while correcting. The vehicle is wide enough to clip
# things well past 22 deg, and the subtended angle grows as range closes
# (~22 deg at 1.0 m, ~34 deg at 0.6 m). 30 catches those without reaching far
# enough sideways to start reading bend geometry as an obstacle.
FRONT_ARC_DEG = 30           # +/- degrees treated as "front path"
SIDE_ARC_DEG = 55            # sector used to pick a dodge direction
ZONE_ARC_DEG = 85            # wide arc used ONLY for "am I beside a building"

# A fixed angular cone is the wrong model for collision checking, and it let
# the buggy drive into a tree: a thin trunk sitting 25 deg off centre falls
# outside a +/-22 deg arc entirely, yet the vehicle is easily wide enough to
# clip it. The angle that matters also grows as range closes - an obstacle
# subtends ~22 deg at 1.0 m but ~34 deg at 0.6 m. So instead of an arc we
# project every return into forward/lateral components and ask the question
# that actually matters: will this hit my body if I keep going straight?
# Nav2 reports this footprint's inscribed radius as 0.402 m, which for a
# longer-than-wide body is its half-width - so the corridor must be wider than
# that or we are checking a path narrower than the vehicle. 0.55 leaves ~15 cm
# of margin, which also covers the lateral drift of a steering correction.
CORRIDOR_HALF_WIDTH = 0.55   # metres; vehicle half-width plus safety margin
CORRIDOR_ARC_DEG = 80        # only returns within this bearing can be ahead
DEBUG_LIDAR = False          # print clearances at 2 Hz for tuning

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
# ARRIVAL IS A DISTANCE, NOT A TIMER.
# The zone images in the README show the zone as a rectangle on the ROAD
# spanning the building's frontage - so reaching it means driving far enough
# ALONG the road to come level with the building. The old 3 s timer was about
# 0.75 m of travel at approach speed, which stopped the buggy ~2.5 m short of
# the frontage on every measured run. We record the pose at first decode and
# drive until we have covered APPROACH_ADVANCE_M.
APPROACH_ADVANCE_M = 3.7     # metres of travel from first decode to arrival
APPROACH_MAX_SEC = 20.0      # hard cap; must exceed ADVANCE_M / SPEED_APPROACH
                             # or the cap fires before arrival can
APPROACH_STALL_SEC = 1.5     # secondary trigger: closing has stopped improving
APPROACH_PROGRESS_M = 0.05   # closing less than this does not count as progress
APPROACH_ARC_DEG = 75        # arc searched for the building we are pulling up to
# Once we commit to a building we do NOT abandon it the moment the code leaves
# frame. At cruise the buggy gets only a short burst of decodes as it comes
# level with a board; if losing freshness cancelled the approach it would
# accelerate away from a building it had already decided to visit. Instead we
# hold the commitment and crawl, which nearly always lets the decode return.
APPROACH_QR_GRACE_SEC = 6.0  # abort window - but ONLY in the early approach
# Measured (Raceway_1): the decode dies at ~2.1 m of a 3.7 m approach, because
# pulling level with the board slides it out of the forward camera. In the
# LATE approach, losing the code is therefore EXPECTED, not evidence of a
# misread - so past the halfway point QR loss stops being an abort reason and
# the remaining distance is covered on odometry alone. The commitment was made
# on a solid decode; ARRIVAL_QR_MAX_AGE still protects the transmit itself.
# This gate must never be TIGHTER than the approach itself is long, or a
# perfectly good arrival gets rejected at the last moment - which is exactly
# what happened at 12.0 s (arrival at qr_age 12.7 s). The approach is already
# bounded by APPROACH_MAX_SEC and aborts on early code loss, so any decode
# still inside the approach window is trustworthy by construction. Set it
# above the cap and let the approach logic be the real filter.
ARRIVAL_QR_MAX_AGE = 25.0    # oldest decode still accepted at the transmit gate
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
MAX_RETRIES = 10             # PDF: "Buggy needs to resend at least 5 messages
                             # with 1 second interval" - and logs of persistent
                             # correct sends earn the team another chance if
                             # the server itself fails. So retry well past 5.
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
# Reversing alone was not enough: the buggy backed off a tree and then drove
# straight into it again, because nothing changed its heading. After backing
# out we therefore hold a steer away from the obstacle for a moment before
# normal control resumes.
ESCAPE_SEC = 1.2             # forced turn-away after reversing
ESCAPE_TURN = 0.70
ESCAPE_SPEED = 0.22
# Odometry here is wheel-derived, so a buggy grinding against a pole with its
# wheels turning reports forward motion it is not making - which blinds the
# distance-based detector to the exact case it exists for. This second detector
# trusts only the LiDAR: if something sits in the collision corridor at contact
# range while we are commanding forward motion, we are stuck, whatever the
# odometry claims.
STUCK_CONTACT_DIST = 0.45    # corridor range that means we are against something
STUCK_CONTACT_SEC = 2.0      # held for this long while driving => stuck

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
# Leaving a Patient Zone before the assignment arrives is explicitly a penalty,
# and the PDF shows the server replying within ~1 s - so a timeout firing means
# something is already badly wrong, and we wait a long time before accepting
# the penalty of moving on. The hospital-side wait has no such stated penalty.
WAIT_ASSIGNMENT_TIMEOUT = 45.0  # patient zone: leaving early is a scored penalty
WAIT_NEXT_TIMEOUT = 25.0        # hospital zone: no equivalent penalty stated
FIRST_PATIENT = "PATIENT_1"  # NXP: "the first Patient will always be by default
                             # patient A ... navigate to patient A as soon as
                             # your buggy is spawned"
TOTAL_PATIENTS = 3
# The timer stops at the third delivery, so parking costs no time percentile.
# The rules allow repeated server interaction and the buggy need not be
# stopped, only inside, when PARKED is sent - so we announce repeatedly while
# traversing the exit to maximise the chance one lands inside the zone.
PARK_ANNOUNCE_EVERY = 3.0
PARK_WINDOW_SEC = 55.0       # server waits 1 minute; stay inside that

# --- payload normalisation ---
CODE_TO_NAME = {
    "A": "PATIENT_1", "B": "PATIENT_2", "C": "PATIENT_3",
    "X": "HOSPITAL_1", "Y": "HOSPITAL_2", "Z": "HOSPITAL_3",
}
NAME_TO_CODE = {v: k for k, v in CODE_TO_NAME.items()}

# WIRE FORMAT. The official Server_Communication PDF shows every buggy message
# as a single letter (msg: "A", "X"). A forum answer says to send the string as
# read by the QR ("{LOC: PATIENT_1}"). These cannot both be right, and guessing
# wrong zeroes the run. So: lead with the PDF's letter, and if the server
# rejects it while we are STILL reading this building's code (proof we have not
# moved, so the zone is not what is wrong), retry in place with the next
# format. The PDF's own "not at correct position" example shows the server
# tolerating a rejected transmission followed by a retry, so this is safe.
# Whichever format is accepted is latched for the rest of the run.
PAYLOAD_CODE = 'CODE'        # single letter, e.g. "A"  (official PDF)
PAYLOAD_RAW = 'RAW'          # verbatim QR text, e.g. "{LOC: PATIENT_1}" (forum)
PAYLOAD_NAME = 'NAME'        # building name only, e.g. "PATIENT_1"
PAYLOAD_FORMATS = [PAYLOAD_CODE, PAYLOAD_RAW, PAYLOAD_NAME]

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
        self.fork_until = 0.0           # fork mode active until this time
        self.fork_take_left = None      # branch latched for the current fork
        self.fork_logged = False
        self.prev_deviation = 0.0
        self.prev_lane_time = None
        self.lane_lost_since = None
        self.lane_lost_yaw = None
        self.front_dist = float('inf')
        self.left_clear = float('inf')
        self.right_clear = float('inf')
        self.nearest_dist = float('inf')
        self.nearest_bearing = 0.0
        self.corridor_dist = float('inf')
        self.corridor_side = 1.0
        self.lane_deviation = 0.0
        self.avoiding_against_lane = False
        self.dodge_side_left = None     # latched dodge direction for this encounter
        self.dodge_until = 0.0
        self.approach_since = None
        self.approach_target = None
        self.approach_best = float('inf')
        self.approach_best_time = 0.0
        self.approach_start_x = 0.0     # pose where the current approach began
        self.approach_start_y = 0.0
        self.arrival_building = None    # what AT_BUILDING should transmit
        self.registered = set()      # patients already transmitted to the server
        self.obstacle_block = False
        self._last_lidar_log = 0.0
        self._last_zone_log = 0.0

        self.last_qr = None
        self.last_qr_time = 0.0
        # Verbatim QR text per building, so the RAW payload format can be sent
        # later without needing the code back in frame.
        self.qr_raw_by_building = {}
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
        self.contact_since = None
        self.recovery_until = 0.0
        self.recovery_return_state = None
        self.last_recovery_time = 0.0
        self.recovery_turn_sign = 1.0

        # ---------------- mission state ----------------
        self.state = State.INIT
        self.target_building = None     # e.g. "PATIENT_1"
        self.assigned_hospital = None
        self.pending_delivery = None    # hospital transmitted, awaiting confirm
        self.delivered = 0              # CONFIRMED deliveries only
        self.payload_index = 0          # index into PAYLOAD_FORMATS
        self.payload_proven = False     # set once the server accepts a payload
        self.last_tx_building = None    # what the last transmission was about
        self.state_entered = time.time()
        self.parked_sent = False
        self.park_started = None
        self.last_park_announce = 0.0

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
        # Once the deliveries are done there is no target at all. Without this
        # the exit run would adopt any patient building it happened to pass,
        # stop, and transmit after the mission had already ended - losing the
        # parking bonus and confusing the server.
        if self.state in (State.SEEK_EXIT, State.PARKING, State.DONE):
            return None
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
        True when we are commanding forward motion but not actually escaping.

        Two independent detectors, because either can be fooled alone: wheel
        odometry lies when the wheels spin against an obstacle, and the LiDAR
        corridor says nothing about a buggy beached on geometry it cannot see.
        """
        if self.target_speed < STUCK_SPEED_MIN:
            # Not trying to move (e.g. waiting in a zone) - not stuck.
            self.last_progress_x = self.pose_x
            self.last_progress_y = self.pose_y
            self.last_progress_time = time.time()
            self.contact_since = None
            return False
        if time.time() - self.last_recovery_time < RECOVERY_COOLDOWN_SEC:
            return False
        # During a committed approach we are DELIBERATELY crawling right next
        # to a building. Slow, close and deliberate is not stuck.
        if self.approach_since is not None:
            self.last_progress_x = self.pose_x
            self.last_progress_y = self.pose_y
            self.last_progress_time = time.time()
            return False

        # The LiDAR contact detector has been REMOVED. It declared "stuck"
        # whenever anything sat inside the corridor closer than 0.45 m for two
        # seconds - and that is precisely what pulling up beside a building
        # looks like. It fired during correct approaches (we now stop at
        # near ~0.65 m), reversed the buggy out of the zone, and left it off
        # the road on the apron. The video-era detector used odometry alone
        # and did not have this failure mode.
        #
        # Not covering ground while commanding motion is the honest signal.
        if not self.have_pose:
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
        self.recovery_turn_sign = (-1.0 if self.left_clear < self.right_clear
                                   else 1.0)
        self.get_logger().warn(
            f"[RECOVERY] stuck detected - reversing out "
            f"(front={self.front_dist:.2f} L={self.left_clear:.2f} "
            f"R={self.right_clear:.2f})")
        self.set_state(State.RECOVERY)

    def drive_recovery(self):
        """Back up, then turn away from the obstacle before resuming."""
        now = time.time()

        if now < self.recovery_until:
            self.set_control(REVERSE_SPEED,
                             REVERSE_TURN * self.recovery_turn_sign)
            return

        # Escape phase: drive forward while holding a turn away from whatever
        # we hit. Without this the buggy reversed off the obstacle and then
        # drove straight back into it, since its heading was unchanged.
        if now < self.recovery_until + ESCAPE_SEC:
            self.set_control(ESCAPE_SPEED,
                             ESCAPE_TURN * self.recovery_turn_sign)
            return

        self.last_recovery_time = now
        self.last_progress_x = self.pose_x
        self.last_progress_y = self.pose_y
        self.last_progress_time = now
        self.contact_since = None
        back_to = self.recovery_return_state or State.SEEK_PATIENT
        self.get_logger().info(f"[RECOVERY] complete -> {back_to.name}")
        self.set_state(back_to)

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
            observed = abs(x2 - x1)
            left_x, right_x = min(x1, x2), max(x1, x2)
            ratio = observed / self.lane_width_px if self.lane_width_px else 1.0

            # --- V-fork: the two boundaries are two different branches ----
            if ratio > FORK_WIDTH_RATIO:
                self.fork_until = now + FORK_HOLD_SEC
            in_fork = now < self.fork_until

            if in_fork:
                want = self.pending_turn if self.pending_turn in (
                    'LEFT', 'RIGHT') else None
                # Latch the branch on entry - re-deciding each frame saws
                # the wheel, because steering toward the wider gap changes
                # which gap is wider.
                if self.fork_take_left is None:
                    if want == 'LEFT':
                        self.fork_take_left = True
                    elif want == 'RIGHT':
                        self.fork_take_left = False
                    else:
                        self.fork_take_left = left_x > (width - right_x)
                take_left = self.fork_take_left

                # Aim at the middle of the chosen opening, directly (no EMA:
                # at a fork the smoother is pure lag).
                aim = (left_x / 2.0) if take_left else (right_x + width) / 2.0
                dev = (half - aim) / half
                self.lane_deviation = dev
                self.lane_turn = max(min(FORK_KP * dev, TURN_MAX), -TURN_MAX)
                self.prev_deviation = dev
                self.prev_lane_time = now
                self.lane_visible = True
                self.lane_lost_since = None

                if not self.fork_logged:
                    self.fork_logged = True
                    self.get_logger().info(
                        f"[FORK] width {observed:.0f}px (ratio {ratio:.2f}) "
                        f"- taking {'LEFT' if take_left else 'RIGHT'}"
                        f"{' (sign)' if want else ' (wider)'} "
                        f"turn={self.lane_turn:+.2f}")
                # A fork consumes the latched sign instruction.
                if want and abs(dev) < 0.15:
                    self.clear_pending_turn("fork branch entered")
                return
            else:
                self.fork_logged = False
                self.fork_take_left = None      # ready for the next fork

            centre = (x1 + x2) / 2.0

            # --- curve bias: undo the perspective drift -------------------
            # Each boundary vector leans in image space when the road bends.
            # Average lean measures the curve; shift the aim point back
            # toward the OUTSIDE by a fraction of it. Zero on a straight.
            lean1 = message.vector_1[0].x - message.vector_1[1].x
            lean2 = message.vector_2[0].x - message.vector_2[1].x
            lean = (lean1 + lean2) / 2.0
            shift = max(min(CURVE_LEAN_GAIN * lean,
                            CURVE_LEAN_MAX_PX), -CURVE_LEAN_MAX_PX)
            centre -= shift

            # Learn the lane width, ignoring implausible observations.
            if 0.25 * width < observed < 1.5 * width:
                r = LANE_WIDTH_LEARN_RATE
                self.lane_width_px = (1.0 - r) * self.lane_width_px + r * observed

        elif count == 1:
            vx = (message.vector_1[0].x + message.vector_1[1].x) / 2.0
            offset = (self.lane_width_px / 2.0) * SINGLE_EDGE_MARGIN
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

        # --- collision corridor -------------------------------------------
        # Forward clearance to anything that would actually strike the body,
        # found by projecting each return into forward/lateral components
        # rather than testing a fixed angular cone. `corridor_side` records
        # which side the closest intruder sits on, so avoidance can steer away
        # from it instead of guessing from side-sector clearances (which mostly
        # measure the buildings flanking the road).
        corridor = float('inf')
        corridor_lat = 0.0
        lo = idx_for(math.radians(-CORRIDOR_ARC_DEG))
        hi = idx_for(math.radians(CORRIDOR_ARC_DEG))
        if lo > hi:
            lo, hi = hi, lo
        for i in range(lo, hi + 1):
            r = ranges[i]
            if r is None or math.isinf(r) or math.isnan(r) or r <= 0.05:
                continue
            b = angle_min + i * angle_inc
            forward = r * math.cos(b)
            if forward <= 0.0:
                continue
            lateral = r * math.sin(b)
            if abs(lateral) < CORRIDOR_HALF_WIDTH and forward < corridor:
                corridor = forward
                corridor_lat = lateral
        self.corridor_dist = corridor
        # +1 => intruder on our left, so we should steer right, and vice versa.
        self.corridor_side = 1.0 if corridor_lat >= 0.0 else -1.0

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
        # Keep the raw text exactly as decoded, for the RAW payload format.
        self.qr_raw_by_building[building] = raw
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

    def dodge_left(self):
        """
        Which way to swerve around something in the forward path, LATCHED.

        Direction logic: side clearance alone is a poor signal, because it
        mostly reports the buildings lining the road - so the roomier side is
        often simply off the track. The LANE therefore gets first say: if the
        lane controller is steering one way and that side has usable room,
        dodge that way, since that direction is both clear and on the road.
        Only when the lane has no opinion, or the side it wants is blocked, do
        we fall back to raw clearance.

        The LATCH is what makes it work. The inputs to that decision change as
        soon as we act on it, so recomputing every tick oscillates and the
        buggy never actually moves sideways. Decide once, hold until clear.
        """
        now = time.time()

        # Still inside a live encounter: hold the committed side.
        if self.dodge_side_left is not None and now < self.dodge_until:
            self.dodge_until = now + AVOID_HOLD_SEC
            return self.dodge_side_left

        wants_left = self.lane_turn > 0.05
        wants_right = self.lane_turn < -0.05

        # If one side is overwhelmingly more open than the other, take it and
        # do not argue - a 7 m gap against a 1.5 m gap is not a close call,
        # whatever the lane happens to prefer at this instant.
        lopsided_left = self.left_clear > AVOID_SIDE_RATIO * self.right_clear
        lopsided_right = self.right_clear > AVOID_SIDE_RATIO * self.left_clear

        if lopsided_left:
            choice = True
        elif lopsided_right:
            choice = False
        elif wants_left and self.left_clear > AVOID_SIDE_MIN:
            choice = True
        elif wants_right and self.right_clear > AVOID_SIDE_MIN:
            choice = False
        else:
            choice = self.left_clear > self.right_clear

        self.dodge_side_left = choice
        self.dodge_until = now + AVOID_HOLD_SEC
        self.get_logger().info(
            f"[AVOID] obstacle at {self.front_dist:.2f} m - dodging "
            f"{'LEFT' if choice else 'RIGHT'} "
            f"(lane_turn={self.lane_turn:+.2f} L={self.left_clear:.2f} "
            f"R={self.right_clear:.2f})")
        return choice

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

    # ---------------- payload format -------------------------------------

    @property
    def payload_style(self):
        return PAYLOAD_FORMATS[self.payload_index]

    def server_payload(self, building):
        """The wire representation of a building under the current format."""
        style = self.payload_style
        if style == PAYLOAD_RAW:
            return self.qr_raw_by_building.get(building, building)
        if style == PAYLOAD_NAME:
            return building
        return NAME_TO_CODE.get(building, building)

    def transmit_building(self, building):
        """Send a building to the server and remember it for a format retry."""
        self.last_tx_building = building
        payload = self.server_payload(building)
        self.get_logger().info(
            f"[SERVER] sending {building} as '{payload}' "
            f"(format={self.payload_style})")
        self.server.send(payload)

    def can_retry_format(self):
        """
        True when an INVALID could plausibly be a payload-format problem.

        Three conditions. The format must not already be proven - once the
        server has accepted something, INVALID means we are outside a zone and
        rewording would waste the retry. We must still be reading the code of
        the building we transmitted, which is what makes an in-place retry
        legitimate rather than a second out-of-zone penalty. And there must be
        an untried format left.
        """
        return (not self.payload_proven
                and self.payload_index < len(PAYLOAD_FORMATS) - 1
                and self.last_tx_building is not None
                and self.qr_is_fresh()
                and self.last_qr == self.last_tx_building)

    def retry_alternate_format(self):
        """Resend the last building in the next payload format, in place."""
        self.payload_index += 1
        payload = self.server_payload(self.last_tx_building)
        self.get_logger().warn(
            f"[SERVER] INVALID for {self.last_tx_building} - still in zone, "
            f"retrying as '{payload}' (format={self.payload_style})")
        self.server.send(payload)
        # Restart the wait clock: the reply we are waiting for is the new one.
        self.state_entered = time.time()

    def mark_payload_proven(self):
        """
        The server acted on something we sent, so this format is correct.

        Guarded on having actually transmitted: the server can address us
        before we have said anything, and treating that as proof would latch a
        format we never tested and disable the fallback for the whole run.
        """
        if self.last_tx_building is None:
            return
        if not self.payload_proven:
            self.payload_proven = True
            self.get_logger().info(
                f"[SERVER] payload format confirmed: {self.payload_style}")

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
            # During parking, INVALID means "not parked correctly" - NOT that a
            # delivery was rejected. Without this check a bad parking position
            # would send us back to a hospital we had already delivered to.
            if self.state in (State.SEEK_EXIT, State.PARKING, State.DONE):
                self.get_logger().warn(
                    "[SERVER] INVALID - not parked correctly, continuing "
                    "to look for the parking area")
                return

            # Before assuming a zone error, consider the payload wording. If we
            # are still reading the code of the building we just transmitted,
            # we are demonstrably where we were when we sent it - the zone is
            # not what changed, so the format is the thing worth varying.
            if self.can_retry_format():
                self.retry_alternate_format()
                return

            # A genuine zone rejection. The penalty is taken, but the points
            # are still available and the PDF's own example shows a rejected
            # transmission followed by a successful retry from inside the zone.
            # So re-approach rather than abandoning the building.
            self.approach_since = None
            self.approach_target = None
            self.arrival_building = None

            if self.pending_delivery:
                hospital = self.pending_delivery
                self.pending_delivery = None
                self.assigned_hospital = hospital
                self.target_building = hospital
                self.get_logger().error(
                    f"[SERVER] INVALID - delivery of {hospital} rejected "
                    f"(outside zone), re-approaching to retry")
                self.set_state(State.SEEK_HOSPITAL)
                return

            if self.target_building and self.target_building.startswith("PATIENT"):
                self.registered.discard(self.target_building)
                self.get_logger().error(
                    f"[SERVER] INVALID - registration of {self.target_building} "
                    f"rejected (outside zone), re-approaching to retry")
                self.set_state(State.SEEK_PATIENT)
                return

            self.get_logger().error("[SERVER] INVALID - resuming search")
            self.set_state(State.SEEK_HOSPITAL if self.assigned_hospital
                           else State.SEEK_PATIENT)
            return

        if upper == "OK":
            # The PDF gives OK two meanings, disambiguated by where we are in
            # the mission. After the THIRD hospital transmission, OK is the
            # challenge-complete confirmation ("CHALLENGE COMPLETED HERE").
            # After PARKED, OK confirms the parking bonus.
            self.mark_payload_proven()
            if self.pending_delivery:
                self.delivered += 1
                self.get_logger().info(
                    f"[MISSION] {self.pending_delivery} CONFIRMED by OK - "
                    f"delivered {self.delivered}/{TOTAL_PATIENTS} - "
                    f"challenge complete, heading for parking bonus")
                self.pending_delivery = None
                self.assigned_hospital = None
                self.target_building = None
                self.set_state(State.SEEK_EXIT)
                return
            self.get_logger().info("[SERVER] parking confirmed OK - run complete")
            self.parked_sent = True
            self.set_state(State.DONE)
            return

        if upper.startswith("HOSPITAL"):
            self.mark_payload_proven()
            self.assigned_hospital = upper
            self.target_building = upper
            self.get_logger().info(f"[MISSION] assigned -> {upper}")
            self.pending_turn = None
            self.latch_turn_for_target()    # we may already hold a useful sign
            self.set_state(State.SEEK_HOSPITAL)
            return

        if upper.startswith("PATIENT"):
            # Receiving a patient is the server's confirmation that the
            # previous delivery landed inside the hospital zone ("If the buggy
            # is in the Hospital wall boundaries and the Hospital is correct,
            # you will receive another Patient"). This is where deliveries are
            # counted - NOT at transmission, which proves nothing.
            self.mark_payload_proven()
            if self.pending_delivery:
                self.delivered += 1
                self.get_logger().info(
                    f"[MISSION] {self.pending_delivery} CONFIRMED - "
                    f"delivered {self.delivered}/{TOTAL_PATIENTS}")
                self.pending_delivery = None
                self.assigned_hospital = None

            self.target_building = upper
            self.get_logger().info(f"[MISSION] next patient -> {upper}")
            self.pending_turn = None
            self.approach_since = None
            self.approach_target = None
            self.arrival_building = None
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
            # The first patient is fixed by the rules, so we start seeking it
            # immediately rather than wandering until some patient happens to
            # appear. This also makes sign boards useful from the very first
            # frame: with a target set, a table containing "A:LEFT" can be
            # acted on at spawn instead of being discarded for want of anything
            # to look up.
            self.target_building = FIRST_PATIENT
            self.get_logger().info(f"[MISSION] first target -> {FIRST_PATIENT}")
            self.latch_turn_for_target()
            self.set_state(State.SEEK_PATIENT)

        elif self.state in (State.SEEK_PATIENT, State.SEEK_HOSPITAL):
            self.drive_seeking()

        elif self.state == State.AT_BUILDING:
            self.drive_stop()
            self.handle_at_building()

        elif self.state == State.WAIT_ASSIGNMENT:
            # Hold position inside the Patient Zone. Leaving before the
            # assignment arrives is explicitly a penalty, and the PDF shows the
            # server replying within about a second - so we are very patient
            # here and treat moving on as a last resort, not a routine timeout.
            self.drive_stop()
            if self.time_in_state() > WAIT_ASSIGNMENT_TIMEOUT:
                self.get_logger().error(
                    f"[MISSION] no assignment in {WAIT_ASSIGNMENT_TIMEOUT:.0f}s "
                    f"- leaving zone (accepting penalty) to keep the run alive")
                self.set_state(State.SEEK_PATIENT)

        elif self.state == State.WAIT_NEXT:
            # Waiting for the delivery to be confirmed - by the next patient
            # (deliveries 1 and 2) or by OK (delivery 3). Both are handled in
            # the server callback; this state only enforces the timeout.
            self.drive_stop()
            if self.time_in_state() > WAIT_NEXT_TIMEOUT:
                self.get_logger().warn(
                    f"[MISSION] no reply in {WAIT_NEXT_TIMEOUT:.0f}s "
                    f"- resuming search")
                self.pending_delivery = None
                self.set_state(State.SEEK_HOSPITAL if self.assigned_hospital
                               else State.SEEK_PATIENT)

        elif self.state == State.SEEK_EXIT:
            # The mission timer stopped at the third delivery, so nothing here
            # costs time percentile - pure bonus. The rules note the buggy need
            # not be stopped, only inside, when PARKED is sent, and repeated
            # server interaction is allowed - so we announce periodically while
            # traversing the exit, so at least one announcement lands inside
            # the parking zone rather than gambling on a single guess.
            self.drive_seeking()
            if self.park_started is None:
                self.park_started = time.time()
                self.get_logger().info(
                    "[MISSION] all deliveries done - heading for parking")

            elapsed = time.time() - self.park_started
            if elapsed > PARK_WINDOW_SEC:
                self.get_logger().info("[MISSION] parking window closed")
                self.set_state(State.DONE)
            elif time.time() - self.last_park_announce > PARK_ANNOUNCE_EVERY:
                self.last_park_announce = time.time()
                self.server.send("PARKED")

        elif self.state == State.PARKING:
            self.drive_stop()

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

        # AVOIDANCE - the video-era front-arc version, restored.
        #
        # This was the regression behind the inner-line cutting. The corridor
        # test asks "will this hit me if I keep going STRAIGHT", projecting a
        # 0.55 m rectangle forward across +/-80 deg. On a bend we are not going
        # straight, so the OUTER boundary of the curve falls inside that
        # rectangle, registers as an obstacle, and the dodge steers away from
        # it - directly into the inner black line. One phantom detection, and
        # the buggy corners itself onto the line it is trying to avoid.
        #
        # The narrow front arc asks a blunter question and does not fire on
        # curve geometry. It detects less, but what it detects is real.
        #
        # NOTE: the corridor is still COMPUTED in lidar_callback - stuck
        # detection reads it. Only the steering response has changed.

        # Emergency: something very close, dead ahead. Steer toward the side
        # with more room but KEEP part of the lane term so we do not rotate
        # blindly across a boundary.
        if self.front_dist < OBSTACLE_STOP_DIST:
            escape = 0.7 if self.dodge_left() else -0.7
            turn = max(min(0.5 * self.lane_turn + escape, TURN_MAX), -TURN_MAX)
            self.set_control(AVOID_BRAKE_SPEED, turn)  # crawl, do not stop
            return

        # Proportional avoidance: ramps from 0 at OBSTACLE_SLOW_DIST to full
        # at OBSTACLE_STOP_DIST. No fixed slap, so driving parallel to a wall
        # that only clips the far edge of the front arc barely perturbs us.
        # Path clear and the hold has expired -> forget the committed side, so
        # the NEXT obstacle gets its own fresh decision.
        if (self.front_dist > OBSTACLE_SLOW_DIST
                and self.dodge_side_left is not None
                and time.time() > self.dodge_until):
            self.dodge_side_left = None

        self.avoiding_against_lane = False
        if self.front_dist < OBSTACLE_SLOW_DIST:
            span = OBSTACLE_SLOW_DIST - OBSTACLE_STOP_DIST
            severity = (OBSTACLE_SLOW_DIST - self.front_dist) / max(span, 1e-3)
            severity = max(0.0, min(1.0, severity))
            bias = OBSTACLE_BIAS_MAX * severity
            bias = bias if self.dodge_left() else -bias

            # Still fighting the lane? Scale the dodge back in proportion to
            # how hard the lane controller is working, keeping a floor so we
            # do not simply drive into the obstacle.
            if bias * self.lane_turn < 0:
                effort = min(abs(self.lane_turn) / LANE_EFFORT_FULL, 1.0)
                bias *= max(AVOID_LANE_FLOOR, 1.0 - effort)
                self.avoiding_against_lane = True

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
            # Origin for the distance measurement: where we first saw the code.
            self.approach_start_x = self.pose_x
            self.approach_start_y = self.pose_y
            self.get_logger().info(f"[APPROACH] closing on {target}")

        if self.approach_since is not None:
            committed = self.approach_target
            since_qr = time.time() - self.last_qr_time
            held = time.time() - self.approach_since

            # How far along the road we have come since the first decode.
            advanced = math.hypot(self.pose_x - self.approach_start_x,
                                  self.pose_y - self.approach_start_y)

            # (advanced is computed above and reused by the speed choice,
            # the abort rules and the arrival test.)
            # Abort rules depend on WHERE we are in the approach. Early on,
            # a long QR dropout means the commitment itself is suspect - give
            # up. Past halfway, the dropout is the geometry working as
            # expected (the board has gone abeam, out of the camera), so only
            # the hard time cap can abort; the rest is odometry.
            early_loss = (since_qr > APPROACH_QR_GRACE_SEC
                          and advanced < 0.5 * APPROACH_ADVANCE_M)
            if early_loss or held > APPROACH_MAX_SEC:
                self.get_logger().warn(
                    f"[APPROACH] abandoning {committed} "
                    f"(no code for {since_qr:.1f}s, held {held:.1f}s)")
                self.approach_since = None
                self.approach_target = None
                self.arrival_building = None
            else:
                if self.nearest_dist < self.approach_best - APPROACH_PROGRESS_M:
                    self.approach_best = self.nearest_dist
                    self.approach_best_time = time.time()

                # NO steering toward the building. This used to lean toward
                # `nearest_bearing`, which once alongside a building IS the
                # building - so it drove the buggy off the road into the wall
                # for the whole final stretch (measured: near fell 1.70 ->
                # 0.90 while the code was already lost, ending with a wheel
                # over the boundary at the stopping position).
                #
                # The zone is a rectangle on the ROAD at the building's
                # frontage, so the correct approach path is simply the lane.
                # Follow it, and let odometry decide when we are level with
                # the building. `turn` is left exactly as the lane controller
                # and avoidance produced it.

                # Speed while approaching. The re-acquire crawl only makes
                # sense EARLY, when a lost code might still come back into
                # frame. Past halfway the board has gone abeam and will not
                # return, so crawling just burns clock - and that is what
                # pushed the decode age past the transmit gate (measured:
                # 1.6 m of crawl at 0.14 m/s = 12 s of ageing). Once
                # committed and past halfway we simply drive the remaining
                # distance at approach speed.
                on_target_now = (self.qr_is_fresh()
                                 and self.last_qr == committed)
                past_half = advanced > 0.5 * APPROACH_ADVANCE_M
                if on_target_now or past_half:
                    approach_speed = SPEED_APPROACH
                else:
                    approach_speed = APPROACH_REACQUIRE_SPEED
                self.set_control(approach_speed, turn)

                stalled = time.time() - self.approach_best_time

                # How far along the road we have come since the first decode.
                # This is the quantity the zone rule is actually written in:
                # the zone is a rectangle at the building's road frontage, so
                # arrival means having driven far enough to be level with it.
                #
                # Distance is the ONLY positive trigger. The wall-proximity
                # gate used to sit here as a secondary and it did real damage:
                # coming alongside drops the nearest range through
                # ZONE_WALL_DIST well before we are level with the frontage,
                # so it fired at 2.65 m of a 3.2 m approach and stopped us
                # short every measured time. A radius test cannot answer a
                # question about a rectangle on the road.
                #
                # The stall trigger stays as a genuine "we are blocked and
                # will not get further" backstop, and only past halfway so an
                # early stall cannot stop us short of the zone.
                arrived = (advanced >= APPROACH_ADVANCE_M
                           or (stalled > APPROACH_STALL_SEC
                               and advanced > 0.5 * APPROACH_ADVANCE_M))

                if arrived:
                    self.get_logger().info(
                        f"[APPROACH] advanced {advanced:.2f} m in {held:.1f}s "
                        f"(near={self.nearest_dist:.2f}) - arrived at {committed}")
                    # Remember the commitment. The code may already be out of
                    # frame at the correct stopping position, so AT_BUILDING
                    # must not re-derive the building from a live read.
                    self.arrival_building = committed
                    self.set_state(State.AT_BUILDING)
                else:
                    now_t = time.time()
                    if now_t - self._last_zone_log > 1.0:
                        self._last_zone_log = now_t
                        self.get_logger().info(
                            f"[ZONE] {committed} advanced={advanced:.2f}/"
                            f"{APPROACH_ADVANCE_M:.1f} near={self.nearest_dist:.2f} "
                            f"qr_age={since_qr:.1f}s held={held:.1f}s")
                return

        # Speed schedule: bleed speed with steering effort, and slow further
        # when lane confidence drops - losing an edge usually means a corner.
        speed = SPEED_CRUISE - TURN_SLOWDOWN_GAIN * abs(turn) * (
            SPEED_CRUISE - SPEED_CORNER)

        # Dodging against the lane is the case that put a wheel over the line.
        # Slowing is the cheapest resolution: the same obstacle clears with a
        # much smaller lateral excursion at low speed.
        if self.avoiding_against_lane:
            speed = min(speed, AVOID_BRAKE_SPEED)

        # A fork needs a tight branch entry in a short distance, which is
        # not possible at cruise. Crawling through it is the difference
        # between making the branch and driving into the divider.
        if time.time() < self.fork_until:
            speed = min(speed, FORK_SPEED)

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
        """We are stopped in the building's zone. Transmit what we committed to."""
        # Use the building the approach committed to, not whatever the camera
        # sees now: at the correct stopping position the board is abeam and
        # often out of frame, so requiring a live read here would reject
        # exactly the position we spent the whole approach reaching.
        building = self.arrival_building or self.last_qr
        self.arrival_building = None

        if building is None:
            self.get_logger().warn(
                "[MISSION] arrived with no building - backing out")
            self.approach_since = None
            self.approach_target = None
            self.set_state(
                State.SEEK_HOSPITAL if self.assigned_hospital
                else State.SEEK_PATIENT)
            return

        # We must still have READ it recently - the honesty check that keeps
        # us from transmitting at a building we never actually reached.
        age = time.time() - self.last_qr_time
        if self.last_qr != building or age > ARRIVAL_QR_MAX_AGE:
            self.get_logger().warn(
                f"[MISSION] no recent read of {building} "
                f"(last={self.last_qr}, age={age:.1f}s) - backing out")
            # CANCEL the approach. Without this the distance condition is
            # still satisfied on the very next control tick, so we re-arrive,
            # get rejected again, and thrash between SEEK and AT_BUILDING at
            # 20 Hz until the approach cap expires.
            self.approach_since = None
            self.approach_target = None
            self.set_state(
                State.SEEK_HOSPITAL if self.assigned_hospital
                else State.SEEK_PATIENT)
            return

        if building.startswith("PATIENT"):
            self.transmit_building(building)
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
            self.transmit_building(building)
            # NOT counted yet. The server confirms a delivery by sending the
            # next patient (deliveries 1-2) or OK (delivery 3), and rejects
            # with INVALID. Counting at transmission would let three rejected
            # transmissions look like a completed mission.
            self.pending_delivery = building
            self.get_logger().info(
                f"[MISSION] delivery of {building} sent - awaiting confirmation")
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