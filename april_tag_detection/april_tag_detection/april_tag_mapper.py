from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf2_geometry_msgs

# where april_tag_detector is publishing detections
INCOMING_TAG_DETECTION_TOPIC = '/apriltag_detection/tag_detection'

GLOBAL_FRAME_ID = "odom" # ideally, this should be changed to "map" when using slam

class TagMapper(Node):

    def __init__(self):
        super().__init__('tag_mapper')

        # Cache: tag_id -> PointStamped (map frame)
        self.tag_cache: dict[int, PointStamped] = {}

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self)

        # Subscriber
        self.sub = self.create_subscription(
            PointStamped,
            INCOMING_TAG_DETECTION_TOPIC,
            self.process_tag_callback,
            10
        )

        # Marker publisher
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/apriltag_markers',
            10
        )

        # Timer to republish markers
        self.create_timer(0.5, self.publish_markers)

    def extract_tag_id(self, msg: PointStamped) -> tuple[str, int]:
        """
        Example frame_id: 'camera_link:=:7'
        Returns original frame id and decoded tag
        """
        frame = msg.header.frame_id
        if ':=:' in frame:
            frame_id_real, tag_id = frame.split(':=:')
            return frame_id_real, int(tag_id)
        else:
            return frame, -1  # unknown

    def process_tag_callback(self, point: PointStamped):

        frame_id, tag_id = self.extract_tag_id(point)
        if tag_id < 0:
            # tag_id is -1 if extraction failed
            self.get_logger().warn("Could not extract tag ID")
            return

        try:
            # Lookup transform: camera_frame → map
            transform = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME_ID,                    # target frame
                frame_id,      # source frame
                Time()
            )

            # Transform the point
            point_in_map = tf2_geometry_msgs.do_transform_point(
                point,
                transform
            )

            # Store in cache
            self.tag_cache[tag_id] = point_in_map
            #self.tag_cache[tag_id] = point  # temporarly store raw point

            self.get_logger().info(
                f"Stored tag {tag_id} at "
                f"({point_in_map.point.x:.2f}, "
                f"{point_in_map.point.y:.2f}, "
                f"{point_in_map.point.z:.2f}) in '{GLOBAL_FRAME_ID}' frame"
                f"(original xyz: {point.point.x:.2f} {point.point.y:.2f} {point.point.z:.2f})"
            )

        except Exception as e:
            self.get_logger().warn(
                f"TF transform failed for tag {tag_id}: {e}"
            )

    def publish_markers(self):
        marker_array = MarkerArray()

        for tag_id, point in self.tag_cache.items():

            marker = Marker()
            marker.header.frame_id = GLOBAL_FRAME_ID #'oakd_link' #GLOBAL_FRAME_ID
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = 'apriltags'
            marker.id = tag_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position = point.point
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15

            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


# ----------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------

def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TagMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
