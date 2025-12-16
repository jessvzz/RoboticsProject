"""
MAKE SURE TO RUN:
sudo apt install ros-jazzy-cv-bridge
sudo apt install ros-jazzy-image-transport

pip install pupil-apriltags
pip install apriltag opencv-python numpy

--------

AprilTag position estimation node (ROS 2 Jazzy, Python).

This node:
- Subscribes to a camera image topic (sensor_msgs/Image)
- Subscribes to camera intrinsics (sensor_msgs/CameraInfo)
- Detects AprilTags in the image
- Publishes the position as geometry_msgs/PointStamped on "/apriltag_detection/tag_detection" (realtive to camera frame)
- Publishes a debug image on "/apriltag_detection/debug_image" with drawn boxes around apriltags (you can view it via rqt)

Orientation of tag is intentionally ignored.
"""

import sys
print(sys.executable)

from typing import Optional, List, cast

import rclpy
from rclpy.node import Node

import numpy as np
import numpy.typing as npt
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

from pupil_apriltags import Detector, Detection

from .utils import rate_limit

# camera topics published by turtlebot
CAMERA_RAW_TOPIC = "/oakd/rgb/preview/image_raw"
CAMERA_INFO_TOPIC = "/oakd/rgb/preview/camera_info"

# link frame of the camera, used for publishing PointStamped
CAMERA_LINK = "oakd_link"

class AprilTagDetectorNode(Node):
    """
    ROS 2 node for estimating the 3D position of AprilTags
    relative to the camera frame.
    """

    def __init__(self) -> None:
        super().__init__("apriltag_detector")

        # --- ROS infrastructure ---
        self._bridge: CvBridge = CvBridge()

        self._image_sub = self.create_subscription(
            Image,
            CAMERA_RAW_TOPIC,
            self.image_callback,
            10,
        )

        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            10,
        )

        self._point_pub = self.create_publisher(
            PointStamped,
            "tag_detection",
            10,
        )

        self._debug_image_pub = self.create_publisher(
            Image,
            "debug_image",
            10,
        )

        # --- AprilTag detector ---
        self._detector: Detector = Detector(
            families="tag36h11",
            nthreads=2,
            quad_decimate=2.0,
            refine_edges=True
        )


        # --- Camera intrinsics (filled from CameraInfo) ---
        self._fx: float = 554.0
        self._fy: float = 554.0
        self._cx: float = 320.0
        self._cy: float = 240.0
        self._received_caminfo = False

        # --- Known physical size of the tag (meters) ---
        self._tag_size_m: float = 0.18  # 0.2m in sdf model - 0.02 white pixel borders

        self.get_logger().info("AprilTag detector node started")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """
        Stores camera intrinsic parameters from CameraInfo.
        Only needs to run once.
        """
        if self._received_caminfo:
            return

        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]

        self._received_caminfo = True
        self.get_logger().info("Camera intrinsics received")

    @rate_limit(50)
    def image_callback(self, msg: Image) -> None:
        """
        Processes incoming images, detects AprilTags,
        and publishes estimated positions.
        """
        # Ensure camera intrinsics are available
        if not self._received_caminfo:
            return
    
        # Convert ROS image → OpenCV grayscale image
        gray_image: np.ndarray = self._bridge.imgmsg_to_cv2(
            msg, desired_encoding="mono8"
        )

        # Convert to color for drawing
        color_image: np.ndarray = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)

        # Detect AprilTags
        # NOTE: return type of detect method is broken, it actually returns a list of detection, not just one Detection object 
        detections: List[Detection] = self._detector.detect(
            gray_image,
            estimate_tag_pose=True,
            tag_size=self._tag_size_m,
            camera_params=(self._fx, self._fy, self._cx, self._cy)
        )
        
        for dec in detections:
            self._publish_tag_position(dec, msg.header)
            self._draw_detection(color_image, dec)

        # Publish debug image
        debug_msg = self._bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        debug_msg.header = msg.header
        self._debug_image_pub.publish(debug_msg)

    # ------------------------------------------------------------------
    # Position estimation
    # ------------------------------------------------------------------

    def _publish_tag_position(self, detection: Detection, header: Header) -> None:
        """
        Estimates the 3D position of a detected AprilTag and publishes it.
        """
        
        ok = isinstance(self._tag_size_m, float) and \
                isinstance(self._fx, float) and \
                isinstance(self._fy, float) and \
                isinstance(self._cx, float) and \
                isinstance(self._cy, float)
        if not ok:
            self.get_logger().error("Camera intrinsics or tag size not set")
            return
        
        # some magic to get the type hints working
        pose_t: npt.NDArray[np.float64]  # shape=(3,1) 
        pose_t = cast(np.ndarray, detection.pose_t)
        if pose_t is None:
            self.get_logger().error("Tag pose could not be estimated")
            return
        
        Y, Z, X = pose_t.flatten()  # 3D position in camera frame
        point_msg = PointStamped()
        point_msg.header = header
        # we are encoding the tag id in the frame_id string field of header, to pass it to the tag mapper (kind of hacky, but works)
        point_msg.header.frame_id = CAMERA_LINK + f":=:{detection.tag_id}"
        point_msg.point.x = float(X)
        point_msg.point.y = -float(Y)
        point_msg.point.z = -float(Z)
        self._point_pub.publish(point_msg)

    # ------------------------------------------------------------------
    # Draw detected tags
    # ------------------------------------------------------------------
    def _draw_detection(self, image: np.ndarray, detection: Detection) -> None:
        """Draw bounding box, center, and ID of the AprilTag on the image."""
        corners = detection.corners.astype(int) # type: ignore
        center = tuple(detection.center.astype(int)) # type: ignore

        # Draw tag outline (green)
        for i in range(4):
            pt1 = tuple(corners[i])
            pt2 = tuple(corners[(i + 1) % 4])
            cv2.line(image, pt1, pt2, (0, 255, 0), 2)

        # Draw center (red)
        cv2.circle(image, center, 4, (0, 0, 255), -1)

        # Draw tag ID (blue)
        cv2.putText(
            image,
            f"ID: {detection.tag_id}",
            (center[0] + 10, center[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            2,
        )



# ----------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
