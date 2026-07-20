# Copyright 2024-2026 NXP
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import os

# HINT: TensorFlow/Keras can be heavy and might not be installed by default.
# We wrap the import in a try-except block so the node runs even if TensorFlow is missing.
# Install it using: pip install tensorflow
try:
    import tensorflow as tf
except ImportError:
    tf = None

class ObjectRecognizer(Node):
    """
    ROS 2 Node that processes raw camera images to recognize traffic sign boards.
    It publishes the detected sign type/labels on the `/sign_board_detection` topic.
    """
    def __init__(self):
        super().__init__('object_recognizer')

        # Subscription for camera images.
        self.subscription_camera = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_image_callback,
            10)

        # Publisher for sign board detection results.
        self.publisher_sign = self.create_publisher(
            String,
            '/sign_board_detection',
            10)

        # Attempt to load the pre-trained Keras model (model.h5) located in the same directory.
        self.model = None
        if tf is not None:
            try:
                dir_path = os.path.dirname(os.path.abspath(__file__))
                model_path = os.path.join(dir_path, 'model.h5')
                if os.path.exists(model_path):
                    self.model = tf.keras.models.load_model(model_path)
                    self.get_logger().info(f"Loaded Keras model from {model_path}")
                else:
                    self.get_logger().warn(f"Model file not found at {model_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load Keras model: {e}")
        else:
            self.get_logger().warn("TensorFlow is not installed. Running in CV/Placeholder mode.")

        self.get_logger().info("Object Recognizer Node started. Waiting for images...")

    def camera_image_callback(self, message):
        """Processes incoming camera frames to classify traffic signs."""
        # Convert compressed image message to OpenCV format
        np_arr = np.frombuffer(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        sign_detected = self.classify_sign(image)

        if sign_detected is not None:
            msg = String()
            msg.data = sign_detected
            self.publisher_sign.publish(msg)
            self.get_logger().info(f"Detected Sign Board: {sign_detected}")

    def classify_sign(self, image):
        """
        Classify traffic sign boards.
        
        OPTIMIZATION HINTS:
        - If TensorFlow is installed, you can pre-process the image (e.g. crop the sign region, 
          resize to 150x150, normalize, expand dimensions) and feed it into `self.model.predict()`.
        - Alternatively, you can use classic Computer Vision techniques:
          1. Color Segmentation: Convert to HSV and threshold for specific sign colors.
          2. Shape Detection: Find contours and approximate polygons.
          3. Template Matching: Match regions of interest against template images of sign boards.
        """
        # Example Keras model prediction template:
        if self.model is not None:
            try:
                # Resize image to match model input dimensions (e.g., 150x150)
                resized_image = cv2.resize(image, (150, 150))
                # Add batch dimension
                image_array = np.expand_dims(resized_image, axis=0) / 255.0  # Normalized
                
                predictions = self.model.predict(image_array, verbose=0)
                # Parse predictions based on your model's classification classes
                # Example:
                # class_idx = np.argmax(predictions[0])
                # if class_idx == 0:
                #     return "STOP_SIGN"
            except Exception as e:
                self.get_logger().debug(f"Inference failed: {e}")

        # Basic OpenCV color/shape detection placeholder code:        
        return None

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
