#!/usr/bin/env python3 
""" 
sudo apt install ros-jazzy-cv-bridge
sudo apt install ros-jazzy-image-transport

pip install pupil-apriltags
pip install apriltag opencv-python numpy


AprilTag position estimation node (ROS 2 Jazzy, Python).

This node:
- Subscribes to a camera image topic (sensor_msgs/Image)
- Subscribes to camera intrinsics (sensor_msgs/CameraInfo)
- Detects AprilTags in the image
- Estimates a rough 3D position of each tag relative to the camera
- Publishes the position as geometry_msgs/PointStamped

Orientation is intentionally ignored.
"""

from typing import Optional, List

import rclpy
from rclpy.node import Node

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

from pupil_apriltags import Detector

class AprilTagPositionNode(Node):
    """
    ROS 2 node for estimating the 3D position of AprilTags
    relative to the camera frame.
    """

    def __init__(self) -> None:
        super().__init__("apriltag_position_node")

        # --- ROS infrastructure ---
        self._bridge: CvBridge = CvBridge()

        self._image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10,
        )

        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera_info",
            self.camera_info_callback,
            10,
        )

        self._point_pub = self.create_publisher(
            PointStamped,
            "/apriltag/position",
            10,
        )

        # --- AprilTag detector ---
        self._detector: Detector = Detector(
            families="tag36h11",
            nthreads=2,
            quad_decimate=2.0,
            refine_edges=True,
        )

        # --- Camera intrinsics (filled from CameraInfo) ---
        self._fx: Optional[float] = None
        self._fy: Optional[float] = None
        self._cx: Optional[float] = None
        self._cy: Optional[float] = None

        # --- Known physical size of the tag (meters) ---
        self._tag_size_m: float = 0.16  # CHANGE to match your tags

        self.get_logger().info("AprilTag position node started")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """
        Stores camera intrinsic parameters from CameraInfo.
        Only needs to run once.
        """
        if self._fx is not None:
            return

        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]

        self.get_logger().info("Camera intrinsics received")

    def image_callback(self, msg: Image) -> None:
        """
        Processes incoming images, detects AprilTags,
        and publishes estimated positions.
        """
        # Ensure camera intrinsics are available
        if self._fx is None:
            return

        # Convert ROS image → OpenCV grayscale image
        gray_image: np.ndarray = self._bridge.imgmsg_to_cv2(
            msg, desired_encoding="mono8"
        )

        # Detect AprilTags
        detections = self._detector.detect(gray_image)

        for detection in detections:
            self._publish_tag_position(detection, msg.header)

    # ------------------------------------------------------------------
    # Position estimation
    # ------------------------------------------------------------------

    def _publish_tag_position(self, detection, header: Header) -> None:
        """
        Estimates the 3D position of a detected AprilTag and publishes it.
        """

        # Pixel coordinates of the tag center
        u: float = float(detection.center[0])
        v: float = float(detection.center[1])

        # Corner pixels of the tag (4x2 array)
        corners: np.ndarray = detection.corners

        # Estimate tag size in pixels (average side length)
        side_lengths: List[float] = [
            float(np.linalg.norm(corners[i] - corners[(i + 1) % 4]))
            for i in range(4)
        ]
        tag_size_px: float = float(np.mean(side_lengths))

        # Reject very small detections (too noisy)
        if tag_size_px < 15.0:
            return

        # Depth estimation (Z)
        # Z = (real_tag_size * focal_length) / observed_pixel_size
        Z: float = (self._tag_size_m * self._fx) / tag_size_px

        # Back-project pixel to 3D camera coordinates
        X: float = (u - self._cx) * Z / self._fx
        Y: float = (v - self._cy) * Z / self._fy

        # Create and publish PointStamped
        point_msg: PointStamped = PointStamped()
        point_msg.header = header
        point_msg.header.frame_id = "camera_link"

        point_msg.point.x = X
        point_msg.point.y = Y
        point_msg.point.z = Z

        self._point_pub.publish(point_msg)


# ----------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = AprilTagPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
