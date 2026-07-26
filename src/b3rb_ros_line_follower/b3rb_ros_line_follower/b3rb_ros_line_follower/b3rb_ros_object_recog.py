# =============================================================================
# b3rb_ros_object_recog.py
# -----------------------------------------------------------------------------
# NXP Cup India 2026 - Autonomous Medical Response
# Team 2694  |  Node: "detect"
#
# PURPOSE
#   Reads the direction sign boards and publishes a ROUTING TABLE telling the
#   mission controller which way to turn for each destination.
#
# WHY A TABLE AND NOT A SINGLE INSTRUCTION
#   The labelled dataset shows a single board carrying SEVERAL destinations at
#   once, e.g. one frame contained A->Left, B->Right, C->Straight, X->Straight.
#   Publishing only "turn left" would throw away most of that. Instead this
#   node publishes every pair it can see, and the controller looks up whichever
#   building it happens to be seeking. One good sighting at a junction can
#   therefore serve the whole mission.
#
#   Published format on /sign_board_detection:
#       "A:LEFT,B:RIGHT,C:STRAIGHT,X:STRAIGHT"
#
# HOW LETTERS AND ARROWS ARE PAIRED
#   The model detects letters (A B C X Y Z) and arrows (Left Right Straight)
#   as SEPARATE objects - it does not associate them. From the dataset the
#   geometry is consistent: the arrow sits directly BELOW its letter, with the
#   x-centres within a few pixels, e.g.
#       A  centre x 286.5      Left      centre x 289.0
#       B  centre x 349.5      Right     centre x 352.5
#   Pairing is therefore: for each letter, take the nearest arrow that lies
#   below it and is horizontally aligned. Tolerances are expressed as multiples
#   of the letter's own box size so they scale with viewing distance.
#
# LETTERBOXING (must match training)
#   The dataset was exported at 512x512 with "Fit (black edges)". The camera
#   publishes 320x240. Feeding a raw 320x240 frame would present the model with
#   a different scale distribution than it trained on, so frames are letterboxed
#   the same way: scaled by 512/320 = 1.6 to 512x384, then padded with 64 px of
#   black top and bottom.
#
# PERFORMANCE
#   Detection is throttled - sign boards do not move, and the controller only
#   needs a fresh reading every few hundred milliseconds. Running the detector
#   on every camera frame would waste most of the CPU budget for no benefit.
#
# GRACEFUL DEGRADATION
#   If the weights file is missing or torch cannot load it, the node logs the
#   reason once and stays alive publishing nothing. A missing model must never
#   crash the stack mid-run; the controller already falls back to exploration
#   when no sign data arrives.
# =============================================================================

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

try:
    import torch
    import torchvision
    from torchvision.models.detection.anchor_utils import AnchorGenerator
    TORCH_AVAILABLE = True
except Exception:
    TORCH_AVAILABLE = False


# =============================================================================
# CONFIG
# =============================================================================

MODEL_FILENAME = 'sign_detector.pt'
LETTERBOX_SIZE = 512          # must match training export
SCORE_THRESHOLD = 0.80      # per-detection confidence floor
DETECT_PERIOD_SEC = 0.20      # ~5 Hz; boards are static, no need for 30 Hz

# Pairing tolerances, expressed relative to the letter's own box so they scale
# with distance. Derived from the dataset: arrow centre sits ~1.8x the letter
# height below it, with x-centres within a fraction of the letter width.
PAIR_MAX_DX_MULT = 2.0        # |dx| < 2.0 * letter_width
PAIR_MAX_DY_MULT = 5.0        # 0 < dy < 5.0 * letter_height

LETTERS = {'A', 'B', 'C', 'X', 'Y', 'Z'}
ARROWS = {'Left', 'Right', 'Straight'}


class ObjectRecognizer(Node):
    """Detects sign boards and publishes a destination -> direction table."""

    def __init__(self):
        super().__init__('object_recognizer')

        self.subscription_camera = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_image_callback,
            10)

        self.publisher_sign = self.create_publisher(
            String, '/sign_board_detection', 10)

        self.model = None
        self.class_names = []
        self.device = torch.device('cpu') if TORCH_AVAILABLE else None
        self._last_detect = 0.0
        self._last_published = None

        self._load_model()

    # ------------------------------------------------------------------ #
    # MODEL
    # ------------------------------------------------------------------ #

    def _candidate_paths(self):
        """Weights ship inside the package, so look next to this module first."""
        here = os.path.dirname(os.path.abspath(__file__))
        return [
            os.path.join(here, MODEL_FILENAME),
            os.path.join(here, '..', MODEL_FILENAME),
            os.path.join(os.getcwd(), MODEL_FILENAME),
        ]

    def _load_model(self):
        if not TORCH_AVAILABLE:
            self.get_logger().warn(
                "torch unavailable - sign detection disabled, controller will "
                "fall back to exploration.")
            return

        path = next((p for p in self._candidate_paths() if os.path.isfile(p)),
                    None)
        if path is None:
            self.get_logger().warn(
                f"{MODEL_FILENAME} not found - sign detection disabled. "
                f"Train it and copy it next to this module.")
            return

        try:
            # map_location='cpu' matters: the evaluation machine may have no
            # GPU, and a CUDA-pinned checkpoint would refuse to load there.
            ckpt = torch.load(path, map_location='cpu')
            self.class_names = ckpt['class_names']

            model = torchvision.models.detection \
                .fasterrcnn_mobilenet_v3_large_320_fpn(
                    weights=None,
                    min_size=ckpt.get('min_size', LETTERBOX_SIZE),
                    max_size=ckpt.get('max_size', LETTERBOX_SIZE),
                    num_classes=ckpt['num_classes'])

            # Anchor ladder must match training or the RPN proposes nothing
            # useful - the glyphs are ~18 px and the stock smallest anchor is 32.
            anchor_sizes = tuple(tuple(s) for s in
                                 ckpt.get('anchor_sizes',
                                          ((8, 16, 32, 64, 128),) * 3))
            aspect_ratios = ((0.5, 1.0, 2.0),) * len(anchor_sizes)
            model.rpn.anchor_generator = AnchorGenerator(anchor_sizes,
                                                         aspect_ratios)

            model.load_state_dict(ckpt['state_dict'])
            model.eval()

            if torch.cuda.is_available():
                self.device = torch.device('cuda')
                model.to(self.device)

            self.model = model
            self.get_logger().info(
                f"Sign detector loaded from {os.path.basename(path)} "
                f"on {self.device} | classes: {self.class_names}")

        except Exception as exc:
            self.get_logger().error(
                f"failed to load sign detector ({exc}) - continuing without it")
            self.model = None

    # ------------------------------------------------------------------ #
    # IMAGE PIPELINE
    # ------------------------------------------------------------------ #

    @staticmethod
    def letterbox(image, size=LETTERBOX_SIZE):
        """
        Scale-preserving fit into size x size with black padding, matching the
        dataset's "Fit (black edges)" export.
        """
        h, w = image.shape[:2]
        scale = min(size / w, size / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)

        canvas = np.zeros((size, size, 3), dtype=image.dtype)
        top = (size - nh) // 2
        left = (size - nw) // 2
        canvas[top:top + nh, left:left + nw] = resized
        return canvas

    def camera_image_callback(self, message):
        if self.model is None:
            return

        now = time.time()
        if now - self._last_detect < DETECT_PERIOD_SEC:
            return
        self._last_detect = now

        np_arr = np.frombuffer(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None:
            return

        boxed = self.letterbox(image)
        rgb = cv2.cvtColor(boxed, cv2.COLOR_BGR2RGB)
        tensor = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0

        with torch.no_grad():
            outputs = self.model([tensor.to(self.device)])[0]

        detections = self._collect(outputs)
        table = self.pair_letters_with_arrows(detections)
        if not table:
            return

        payload = ','.join(f"{k}:{v.upper()}" for k, v in sorted(table.items()))
        msg = String()
        msg.data = payload
        self.publisher_sign.publish(msg)

        if payload != self._last_published:
            self.get_logger().info(f"[SIGN] {payload}")
            self._last_published = payload

    def _collect(self, outputs):
        """Filter by score and attach human-readable class names."""
        result = []
        boxes = outputs['boxes'].cpu().numpy()
        labels = outputs['labels'].cpu().numpy()
        scores = outputs['scores'].cpu().numpy()

        for box, label, score in zip(boxes, labels, scores):
            if score < SCORE_THRESHOLD:
                continue
            idx = int(label) - 1                # label 0 is background
            if idx < 0 or idx >= len(self.class_names):
                continue
            x1, y1, x2, y2 = box
            result.append({
                'name': self.class_names[idx],
                'score': float(score),
                'cx': (x1 + x2) / 2.0,
                'cy': (y1 + y2) / 2.0,
                'w': x2 - x1,
                'h': y2 - y1,
            })
        return result

    # ------------------------------------------------------------------ #
    # PAIRING
    # ------------------------------------------------------------------ #

    def pair_letters_with_arrows(self, detections):
        """
        Associate each destination letter with the arrow beneath it.

        Returns {'A': 'Left', 'B': 'Right', ...}. An arrow is consumed by at
        most one letter so two letters cannot claim the same arrow.
        """
        letters = [d for d in detections if d['name'] in LETTERS]
        arrows = [d for d in detections if d['name'] in ARROWS]
        if not letters or not arrows:
            return {}

        table = {}
        used = set()

        # Highest-confidence letters get first pick of the arrows.
        for letter in sorted(letters, key=lambda d: -d['score']):
            max_dx = PAIR_MAX_DX_MULT * letter['w']
            max_dy = PAIR_MAX_DY_MULT * letter['h']

            best, best_cost = None, None
            for i, arrow in enumerate(arrows):
                if i in used:
                    continue
                dx = abs(arrow['cx'] - letter['cx'])
                dy = arrow['cy'] - letter['cy']       # must be positive: below
                if dy <= 0 or dy > max_dy or dx > max_dx:
                    continue
                # Prefer horizontal alignment over vertical closeness: letters
                # on the same board sit in a row, so dx is the discriminator.
                cost = dx * 2.0 + dy
                if best_cost is None or cost < best_cost:
                    best, best_cost = i, cost

            if best is not None:
                used.add(best)
                table[letter['name']] = arrows[best]['name']

        return table


def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()