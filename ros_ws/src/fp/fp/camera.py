import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Camera(Node):
    CAPTURE_PERIOD = 1 / 30

    def __init__(self) -> None:
        super().__init__("camera")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_calibration", "camera_calibration.npz")

        camera_index = (
            self.get_parameter("camera_index").get_parameter_value().integer_value
        )
        camera_calibration = str(
            self.get_parameter("camera_calibration").get_parameter_value().string_value
        )

        self.image_publisher = self.create_publisher(Image, "/camera/image", 10)

        self.capture = cv2.VideoCapture(camera_index)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.bridge = CvBridge()

        self.camera_matrix, self.distortion = self.load_camera_calibration(
            camera_calibration
        )

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.capture_timer = self.create_timer(self.CAPTURE_PERIOD, self.process_frame)
        self.get_logger().info(
            f"Camera initialized | camera_index: {camera_index}"
            f" | calibration: {camera_calibration}"
        )

    def load_camera_calibration(self, calibration_path: str):
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

    def process_frame(self) -> None:
        ret, frame = self.capture.read()
        if not ret:
            self.get_logger().error("Failed to capture image.")
            return

        if self.camera_matrix is not None and self.distortion is not None:
            newcamereamtx, _ = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix,
                self.distortion,
                frame.shape[1::-1],
                1,
                frame.shape[1::-1],
            )
            frame = cv2.undistort(
                frame, self.camera_matrix, self.distortion, None, newcamereamtx
            )

        stamp = self.get_clock().now().to_msg()
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = "camera_link"
        self.image_publisher.publish(image_msg)

    def destroy_node(self):
        self.capture.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
