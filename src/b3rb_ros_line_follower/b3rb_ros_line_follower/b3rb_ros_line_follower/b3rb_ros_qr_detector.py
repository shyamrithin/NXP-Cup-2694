# =============================================================================
# b3rb_ros_qr_detector.py
# -----------------------------------------------------------------------------
# NXP Cup India 2026 - Autonomous Medical Response
# Node: "qr_detect"
#
# PURPOSE
#   Decodes the QR codes mounted on patient and hospital buildings and
#   republishes the payload (e.g. "{LOC: HOSPITAL_1}") on /qr_detection.
#
# WHY THIS DIFFERS FROM THE SHIPPED SKELETON
#   The simulated camera publishes 320x240 frames. At realistic approach
#   distances the QR occupies only ~60-110 px, which is below the reliable
#   working range of cv2.QRCodeDetector - it receives frames but returns no
#   decode. This version therefore:
#
#     1. Uses pyzbar as the primary decoder (markedly better on small, slightly
#        skewed codes) and keeps cv2.QRCodeDetector as a fallback.
#     2. Upscales the frame before decoding, which materially improves the hit
#        rate on distant codes at negligible cost for 320x240 input.
#     3. Also attempts a contrast-normalised grayscale pass, which helps when
#        the board is washed out or in shadow.
#
# ROBUSTNESS NOTE (important for evaluation)
#   pyzbar is a ctypes wrapper around the system library libzbar0. That is an
#   apt package, not a pip one, so it may be absent on the evaluation machine.
#   The import is therefore guarded and the node degrades to cv2 rather than
#   crashing on startup - a missing shared library must never take the QR
#   pipeline down mid-run.
#
# DIAGNOSTICS
#   Set DEBUG_FRAMES = True to log a frame counter every few seconds. This
#   distinguishes "no frames arriving" from "frames arriving but not decoding",
#   which are very different faults.
# =============================================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np

# --- Guarded pyzbar import: never let a missing native lib kill the node. ---
try:
    from pyzbar import pyzbar
    PYZBAR_AVAILABLE = True
except Exception:
    pyzbar = None
    PYZBAR_AVAILABLE = False

# =============================================================================
# CONFIG
# =============================================================================

UPSCALE = 2.0            # decode-time upscale factor for small/distant codes
DEBUG_FRAMES = True      # log frame/decode counters while tuning
DEBUG_PERIOD = 3.0       # seconds between diagnostic logs
REPEAT_LOG_SUPPRESS = True   # only log when the decoded value changes


class QRDetector(Node):
    """Processes camera frames and publishes any decoded QR payload."""

    def __init__(self):
        super().__init__('qr_detector')

        self.subscription_camera = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_image_callback,
            10)

        self.publisher_qr = self.create_publisher(String, '/qr_detection', 10)

        self._cv_detector = cv2.QRCodeDetector()
        self._frames = 0
        self._decodes = 0
        self._last_payload = None
        self._last_debug = self.get_clock().now()

        backend = "pyzbar (+cv2 fallback)" if PYZBAR_AVAILABLE else "cv2 only"
        self.get_logger().info(f"QR Detector started. Backend: {backend}")
        if not PYZBAR_AVAILABLE:
            self.get_logger().warn(
                "pyzbar unavailable (is libzbar0 installed?) - decode range "
                "will be noticeably shorter.")

    # -------------------------------------------------------------------------

    def camera_image_callback(self, message):
        np_arr = np.frombuffer(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None:
            return

        self._frames += 1

        payload = self.detect_qr_code(image)

        if payload:
            self._decodes += 1
            msg = String()
            msg.data = payload
            self.publisher_qr.publish(msg)

            if not REPEAT_LOG_SUPPRESS or payload != self._last_payload:
                self.get_logger().info(f"Published QR Data: {payload}")
            self._last_payload = payload
        else:
            self._last_payload = None

        self._maybe_log_diagnostics()

    def _maybe_log_diagnostics(self):
        if not DEBUG_FRAMES:
            return
        now = self.get_clock().now()
        elapsed = (now - self._last_debug).nanoseconds / 1e9
        if elapsed >= DEBUG_PERIOD:
            self.get_logger().info(
                f"[QR-DIAG] frames={self._frames} decodes={self._decodes}")
            self._last_debug = now

    # -------------------------------------------------------------------------

    def detect_qr_code(self, image):
        """
        Try progressively more aggressive strategies, cheapest first.
        Returns the decoded string, or None.
        """
        # Upscaling is the single biggest win for small codes.
        if UPSCALE and UPSCALE != 1.0:
            big = cv2.resize(image, None, fx=UPSCALE, fy=UPSCALE,
                             interpolation=cv2.INTER_CUBIC)
        else:
            big = image

        gray = cv2.cvtColor(big, cv2.COLOR_BGR2GRAY)

        # --- 1. pyzbar on the upscaled grayscale image ---
        if PYZBAR_AVAILABLE:
            result = self._try_pyzbar(gray)
            if result:
                return result

            # --- 2. pyzbar on a contrast-normalised copy ---
            equalised = cv2.equalizeHist(gray)
            result = self._try_pyzbar(equalised)
            if result:
                return result

        # --- 3. OpenCV fallback ---
        try:
            data, bbox, _ = self._cv_detector.detectAndDecode(big)
            if bbox is not None and data:
                return data
        except Exception as exc:
            self.get_logger().debug(f"cv2 QR detection failed: {exc}")

        return None

    def _try_pyzbar(self, gray_image):
        try:
            for obj in pyzbar.decode(gray_image):
                if obj.data:
                    return obj.data.decode('utf-8', errors='replace')
        except Exception as exc:
            self.get_logger().debug(f"pyzbar decode failed: {exc}")
        return None


# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()
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