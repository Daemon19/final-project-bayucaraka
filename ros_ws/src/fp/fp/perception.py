from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


class Perception(Node):
    CAPTURE_PERIOD = 1 / 30

    def __init__(self) -> None:
        super().__init__("perception")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_calibration", "camera-calibration.npz")
        self.declare_parameter("homography_path", "homography_cm.npy")
        self.declare_parameter("payload_aruco_id", 39)
        self.declare_parameter("dropping_zone_aruco_id", 26)

        camera_index = int(self.get_parameter("camera_index").value)
        camera_calibration = str(self.get_parameter("camera_calibration").value)
        homography_path = Path(str(self.get_parameter("homography_path").value))
        self.payload_aruco_id = int(self.get_parameter("payload_aruco_id").value)
        self.dropping_zone_aruco_id = int(
            self.get_parameter("dropping_zone_aruco_id").value
        )

        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.pixel_payload_pub = self.create_publisher(
            PointStamped, "/aruco/payload/pixel", 10
        )
        self.ground_payload_pub = self.create_publisher(
            PointStamped, "/aruco/payload/ground", 10
        )
        self.pixel_dropping_zone_pub = self.create_publisher(
            PointStamped, "/aruco/dropping_zone/pixel", 10
        )
        self.ground_dropping_zone_pub = self.create_publisher(
            PointStamped, "/aruco/dropping_zone/ground", 10
        )

        self.capture = cv2.VideoCapture(camera_index)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.bridge = CvBridge()

        self.camera_matrix, self.distortion = self.load_camera_calibration(
            camera_calibration
        )

        self.homography = self.load_homography(homography_path)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.capture_timer = self.create_timer(self.CAPTURE_PERIOD, self.process_frame)
        self.get_logger().info(
            f"Perception initialized | camera_index: {camera_index}"
            f" | calibration: {camera_calibration} | homography: {homography_path}"
            f" | payload_aruco_id: {self.payload_aruco_id} | dropping_zone_aruco_id: {self.dropping_zone_aruco_id}"
        )

    def load_camera_calibration(self, calibration_path: str) -> None:
        try:
            calibration_data = np.load(calibration_path)
            camera_matrix = calibration_data["camera_matrix"]
            distortion = calibration_data["distortion"]
            self.get_logger().info(
                f"Loaded camera calibration from: {calibration_path}"
            )
            return camera_matrix, distortion
        except Exception as error:
            self.get_logger().error(
                f"Failed to load camera calibration '{calibration_path}': {error}. Publishing raw frames."
            )
            return None, None

    def load_homography(self, homography_path: Path) -> np.ndarray | None:
        try:
            data = np.load(homography_path)
            homography = data
            homography = np.asarray(homography, dtype=np.float64)
            self.get_logger().info(f"Loaded homography from: {homography_path}")
            return homography
        except Exception as error:
            self.get_logger().error(
                f"Failed to load homography '{homography_path}': {error}. Ground point publishing disabled."
            )
            return None

    def process_frame(self) -> None:
        ret, frame = self.capture.read()
        if not ret:
            self.get_logger().error("Failed to capture image.")
            return

        if self.camera_matrix is not None and self.distortion is not None:
            frame = cv2.undistort(frame, self.camera_matrix, self.distortion)

        stamp = self.get_clock().now().to_msg()

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = "camera_link"
        self.image_pub.publish(image_msg)

        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return

        ids = ids.flatten()

        for index, marker_id in enumerate(ids):
            if marker_id == self.payload_aruco_id:
                pixel_pub = self.pixel_payload_pub
                ground_pub = self.ground_payload_pub
            elif marker_id == self.dropping_zone_aruco_id:
                pixel_pub = self.pixel_dropping_zone_pub
                ground_pub = self.ground_dropping_zone_pub
            else:
                continue

            pixel_x, pixel_y = self.get_aruco_center(corners[index])
            self.publish_center_point(pixel_pub, pixel_x, pixel_y, stamp)

            if self.homography is not None:
                ground_x, ground_y = self.transform_pixel_to_ground(pixel_x, pixel_y)
                self.publish_center_point(ground_pub, ground_x, ground_y, stamp)

            cv2.circle(frame, (int(pixel_x), int(pixel_y)), 5, (0, 255, 0), -1)
            cv2.imshow("Aruco Center", frame)
            cv2.waitKey(1)

    def transform_pixel_to_ground(self, pixel_x, pixel_y):
        point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(point, self.homography)
        return float(transformed[0][0][0]), float(transformed[0][0][1])

    def get_aruco_center(self, corners) -> tuple[float, float]:
        marker_corners = corners.reshape(4, 2)
        center = marker_corners.mean(axis=0)
        return float(center[0]), float(center[1])

    def publish_center_point(self, pub, centerx, centery, stamp):
        pixel_msg = PointStamped()
        pixel_msg.header.stamp = stamp
        pixel_msg.header.frame_id = "camera_link"
        pixel_msg.point.x = centerx
        pixel_msg.point.y = centery
        pixel_msg.point.z = 0.0
        pub.publish(pixel_msg)

    def destroy_node(self):
        cv2.destroyAllWindows()
        self.capture.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Perception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
