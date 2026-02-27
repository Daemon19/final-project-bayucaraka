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
        self.declare_parameter("aruco_id", 39)

        camera_index = int(self.get_parameter("camera_index").value)
        camera_calibration = str(self.get_parameter("camera_calibration").value)
        homography_path = Path(str(self.get_parameter("homography_path").value))
        self.aruco_id = int(self.get_parameter("aruco_id").value)

        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.pixel_pub = self.create_publisher(PointStamped, "/aruco/pixel_center", 10)
        self.ground_pub = self.create_publisher(
            PointStamped, "/aruco/ground_center", 10
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
            f"Perception initialized | camera_index: {camera_index} | calibration: {camera_calibration} | homography: {homography_path} | aruco_id: {self.aruco_id}"
        )

    def load_camera_calibration(self, calibration_path: str) -> None:
        try:
            calibration_data = np.load(calibration_path)
            camera_matrix = calibration_data["camera-matrix"]
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
            if int(marker_id) != self.aruco_id:
                continue

            marker_corners = corners[index].reshape(4, 2)
            center = marker_corners.mean(axis=0)
            pixel_x, pixel_y = float(center[0]), float(center[1])

            pixel_msg = PointStamped()
            pixel_msg.header.stamp = stamp
            pixel_msg.header.frame_id = image_msg.header.frame_id
            pixel_msg.point.x = pixel_x
            pixel_msg.point.y = pixel_y
            pixel_msg.point.z = 0.0
            self.pixel_pub.publish(pixel_msg)

            if self.homography is not None:
                pixel = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
                ground = cv2.perspectiveTransform(pixel, self.homography).reshape(2)

                ground_msg = PointStamped()
                ground_msg.header = pixel_msg.header
                ground_msg.point.x = float(ground[0])
                ground_msg.point.y = float(ground[1])
                ground_msg.point.z = 0.0
                self.ground_pub.publish(ground_msg)

            cv2.circle(frame, (int(pixel_x), int(pixel_y)), 5, (0, 255, 0), -1)
            cv2.imshow("Aruco Center", frame)
            cv2.waitKey(1)

            break

    def destroy_node(self):
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
