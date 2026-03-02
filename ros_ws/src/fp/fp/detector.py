from pathlib import Path

import cv2
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image


class Detector(Node):
    def __init__(self) -> None:
        super().__init__("detector")

        self.declare_parameter("homography_path", "homography.npy")
        self.declare_parameter("topic_names", ["aruco"])
        self.declare_parameter("aruco_ids", [1])
        self.declare_parameter("input_topic", "/camera/image")

        homography_path = Path(str(self.get_parameter("homography_path").value))
        topic_names: list[str] = list(
            self.get_parameter("topic_names").get_parameter_value().string_array_value
        )
        aruco_ids: list[int] = list(
            self.get_parameter("aruco_ids").get_parameter_value().integer_array_value
        )
        input_topic = str(
            self.get_parameter("input_topic").get_parameter_value().string_value
        )

        self.homography = self.load_homography(homography_path)
        self.topic_publishers = self.create_publishers(topic_names, aruco_ids)
        self.camera_subscriber = self.create_subscription(
            Image, input_topic, self.frame_callback, 10
        )

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.get_logger().info(
            f"Detector initialized | "
            f" | homography: {homography_path}"
            f" | topics: {topic_names} | aruco_ids: {aruco_ids}"
        )

    def frame_callback(self, msg: Image) -> None:
        frame = cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return

        ids = ids.flatten()

        detected_ids = set()
        for index, marker_id in enumerate(ids):
            if marker_id not in self.topic_publishers.keys():
                continue
            if marker_id in detected_ids:
                continue
            detected_ids.add(marker_id)

            pixel_x, pixel_y = self.get_aruco_center(corners[index])
            if self.homography is not None:
                ground_x, ground_y = self.transform_pixel_to_ground(pixel_x, pixel_y)
            else:
                ground_x, ground_y = pixel_x, pixel_y

            self.publish_stamped_point(
                self.topic_publishers[marker_id], ground_x, ground_y, msg.header.stamp
            )

            cv2.circle(frame, (int(pixel_x), int(pixel_y)), 5, (0, 255, 0), -1)
            cv2.imshow("Aruco Center", frame)
            cv2.waitKey(1)

    def create_publishers(self, topic_names: list[str], aruco_ids: list[int]):
        if len(topic_names) != len(aruco_ids):
            self.get_logger().error(
                "The number of topic names must match the number of ArUco IDs."
            )
            topic_names = []
            aruco_ids = []

        publishers = {
            aruco_id: self.create_publisher(PointStamped, f"/detection/{topic}", 10)
            for aruco_id, topic in zip(aruco_ids, topic_names)
        }

        return publishers

    def load_homography(self, homography_path: Path) -> np.ndarray | None:
        try:
            data = np.load(homography_path)
            homography = np.asarray(data, dtype=np.float64)
            self.get_logger().info(f"Loaded homography from: {homography_path}")
            return homography
        except Exception as error:
            self.get_logger().error(
                f"Failed to load homography '{homography_path}': {error}. Publishing pixel coordinates instead."
            )
            return None

    def transform_pixel_to_ground(self, pixel_x, pixel_y):
        point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(point, self.homography)
        return float(transformed[0][0][0]), float(transformed[0][0][1])

    def get_aruco_center(self, corners) -> tuple[float, float]:
        marker_corners = corners.reshape(4, 2)
        center = marker_corners.mean(axis=0)
        return float(center[0]), float(center[1])

    def publish_stamped_point(self, pub, centerx, centery, stamp):
        pixel_msg = PointStamped()
        pixel_msg.header.stamp = stamp
        pixel_msg.header.frame_id = "camera_link"
        pixel_msg.point.x = centerx
        pixel_msg.point.y = centery
        pixel_msg.point.z = 0.0
        pub.publish(pixel_msg)

    def destroy_node(self):
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
